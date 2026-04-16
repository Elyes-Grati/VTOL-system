/*
 * =============================================================================
 * VTOL Helicopter Rig — Arduino Firmware
 * =============================================================================
 *
 * This firmware runs on an Arduino Uno (ATmega328P) and serves as the
 * embedded I/O layer for the VTOL rig control system.
 *
 * Responsibilities:
 *   - Read elevation angle and rate from ADXL345 (accelerometer) and
 *     ITG-3205 (gyroscope) over I2C
 *   - Read roll and yaw angles from two analog potentiometers (A0, A1)
 *   - Estimate roll rate and yaw rate via filtered finite differences
 *   - Transmit the 6-element state vector to LabVIEW over serial at 100 Hz
 *   - Receive ESC commands from LabVIEW and write them to both brushless ESCs
 *
 * The LQR-I control law is NOT executed here. It runs on the LabVIEW host,
 * which closes the loop by sending computed PWM values back to this firmware.
 *
 * Serial protocol (230400 baud, newline-terminated):
 *   TX (Arduino → PC):   -pitch,roll,-yaw,gy,filteredRollSpeed,-filteredYawSpeed\n
 *   RX (PC → Arduino):   <esc1_us>,<esc2_us>\n
 *
 * Hardware connections:
 *   ADXL345  : I2C (SDA=A4, SCL=A5), address 0x53
 *   ITG-3205 : I2C (SDA=A4, SCL=A5), address 0x68
 *   Pot Roll : Analog A0
 *   Pot Yaw  : Analog A1
 *   ESC 1    : PWM pin 9  (motor M1)
 *   ESC 2    : PWM pin 10 (motor M2)
 *
 * Authors: Grati Elyes, Njeh Oussema, Snoun Ferid, Khelil Souheib
 * Class:   IIA4
 * =============================================================================
 */

#include <Servo.h>
#include <Wire.h>
#include <math.h>

// =============================================================================
// I2C Device Addresses
// =============================================================================
#define ACCEL_ADDR  0x53   // ADXL345 accelerometer
#define GYRO_ADDR   0x68   // ITG-3205 gyroscope

// =============================================================================
// ESC / Servo Configuration
// =============================================================================
Servo esc1, esc2;

const int ESC1_PIN = 9;
const int ESC2_PIN = 10;
const int ESC_MIN  = 1000;   // Minimum pulse width (µs) — motor stopped
const int ESC_MAX  = 1700;   // Maximum pulse width (µs) — full throttle

// =============================================================================
// Signal Processing Constants
// =============================================================================

// Gyroscope sensitivity: ITG-3205 full-scale ±2000 °/s → 14.375 LSB/(°/s)
// Pre-inverted to replace division with multiplication in the hot path
const float GYRO_SCALE   = 1.0 / 14.375;

// Low-pass filter coefficient for velocity estimation (α = 0.15)
// Higher α → more responsive but noisier; lower α → smoother but slower
const float LP_ALPHA     = 0.15f;
const float LP_ALPHA_INV = 0.85f;   // Pre-computed (1 - LP_ALPHA)

// Deadband applied before velocity filter to suppress ADC quantization noise
// Readings below this threshold are treated as zero movement
const float DEADBAND     = 0.002f;  // radians

// ADC to radians conversion for potentiometers
// Range: 0–1023 ADC → 0–(3π/2) radians = 4.7123 rad total span
// Pre-computed: 4.7123 / 1024 = 0.004602 rad/LSB
const float ADC_SCALE    = 4.7123f / 1024.0f;

// =============================================================================
// Calibration Offsets (set during startup calibration)
// =============================================================================
float pitchOffset = 0;
float rollOffset  = 2.91f;   // Pre-set approximate center; refined at startup
float yawOffset   = 0;
float gxOffset    = 0;
float gyOffset    = 0;

// =============================================================================
// Low-Level I2C Helper
// =============================================================================
/*
 * Writes a single byte to a register on an I2C device.
 * Declared inline to eliminate function call overhead on the hot path.
 */
inline void writeTo(uint8_t device, uint8_t address, uint8_t val) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}

// =============================================================================
// Sensor Read Functions
// =============================================================================

/*
 * Read 3-axis raw acceleration from ADXL345.
 * Register 0x32 is the first data register (X0); the next 5 bytes follow.
 * Data format: little-endian signed 16-bit integers.
 */
void readAccel(float *x, float *y, float *z) {
  Wire.beginTransmission(ACCEL_ADDR);
  Wire.write(0x32);
  Wire.endTransmission();
  Wire.requestFrom(ACCEL_ADDR, 6);
  if (Wire.available() == 6) {
    *x = (float)(int16_t)(Wire.read() | (Wire.read() << 8));
    *y = (float)(int16_t)(Wire.read() | (Wire.read() << 8));
    *z = (float)(int16_t)(Wire.read() | (Wire.read() << 8));
  }
}

/*
 * Read 3-axis angular rate from ITG-3205.
 * Register 0x1D is the first gyro data register; data is big-endian.
 * Apply sensitivity scale factor to convert LSB → degrees/second.
 */
void readGyro(float *x, float *y, float *z) {
  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(0x1D);
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDR, 6);
  if (Wire.available() == 6) {
    *x = (float)(int16_t)((Wire.read() << 8) | Wire.read()) * GYRO_SCALE;
    *y = (float)(int16_t)((Wire.read() << 8) | Wire.read()) * GYRO_SCALE;
    *z = (float)(int16_t)((Wire.read() << 8) | Wire.read()) * GYRO_SCALE;
  }
}

// =============================================================================
// Startup Calibration
// =============================================================================
/*
 * Averages 100 samples from all sensors to establish bias offsets.
 * The rig must be stationary at the equilibrium position during calibration.
 * Offsets are stored globally and subtracted from all subsequent readings.
 */
void calibrate() {
  float sumP = 0, sumGX = 0, sumGY = 0;
  float sumR = 0, sumY  = 0;

  for (int i = 0; i < 100; i++) {
    float ax, ay, az, gx, gy, gz;
    readAccel(&ax, &ay, &az);
    readGyro(&gx, &gy, &gz);

    // Elevation (pitch) from accelerometer: atan2 gives angle in radians
    // sqrtf/atan2f: single-precision variants, faster on AVR
    sumP  += atan2f(-ax, sqrtf(ay * ay + az * az));
    sumR  += analogRead(A0) * ADC_SCALE;
    sumY  += analogRead(A1) * ADC_SCALE;
    sumGX += gx;
    sumGY += gy;
  }

  // Multiply by 1/100 instead of dividing — multiplication is faster on AVR
  pitchOffset = sumP  * 0.01f;
  rollOffset  = sumR  * 0.01f;
  yawOffset   = sumY  * 0.01f;
  gxOffset    = sumGX * 0.01f;
  gyOffset    = sumGY * 0.01f;
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  Wire.begin();
  Serial.begin(230400);   // High baud rate minimizes serial blocking at 100 Hz

  // --- Initialize ADXL345 ---
  // Register 0x2D (Power Control): set bit 3 (Measure mode)
  writeTo(ACCEL_ADDR, 0x2D, 8);

  // --- Initialize unknown device 0x0D ---
  // Register 0x09, value 0x01 (device-specific initialization)
  writeTo(0x0D, 0x09, 0x01);

  // --- Initialize ITG-3205 ---
  // Register 0x3E (Power Management): clock source = PLL with X gyro reference
  writeTo(GYRO_ADDR, 0x3E, 0x01);
  // Register 0x16 (DLPF, Full Scale): FS_SEL=3 (±2000°/s), DLPF=0 (256 Hz)
  writeTo(GYRO_ADDR, 0x16, 0x18);

  delay(500);        // Allow sensors to stabilize
  calibrate();       // Establish bias offsets

  // --- ESC Initialization Sequence ---
  // ESC controllers require a calibration pulse sequence at power-on.
  // This sequence communicates the throttle range to the ESC firmware.
  esc1.attach(ESC1_PIN);
  esc2.attach(ESC2_PIN);

  esc1.writeMicroseconds(ESC_MAX);   // Step 1: full throttle → arm
  esc2.writeMicroseconds(ESC_MAX);
  delay(3000);
  Serial.println("ESC armed");

  esc1.writeMicroseconds(ESC_MIN);   // Step 2: zero throttle → confirm range
  esc2.writeMicroseconds(ESC_MIN);
  delay(3000);
  Serial.println("ESC ready");
}

// =============================================================================
// Main Control Loop
// =============================================================================
void loop() {
  unsigned long loopStart = micros();   // For loop timing diagnostics

  static unsigned long lastMicros      = 0;
  static float lastRoll                = 0;
  static float filteredRollSpeed       = 0;
  static float lastYaw                 = 0;
  static float filteredYawSpeed        = 0;

  unsigned long currentMicros = micros();

  // -------------------------------------------------------------------------
  // Execute sensor read and state transmission at exactly 100 Hz (10 ms)
  // -------------------------------------------------------------------------
  if (currentMicros - lastMicros >= 10000) {

    // Pre-invert dt: dt_inv = 1/dt in seconds^-1
    // Multiplying by dt_inv replaces division, which is slower on AVR
    float dt_inv   = 1000000.0f / (float)(currentMicros - lastMicros);
    lastMicros     = currentMicros;

    // --- Read raw sensor data ---
    float ax, ay, az, rawGX, rawGY, rawGZ;
    readAccel(&ax, &ay, &az);
    readGyro(&rawGX, &rawGY, &rawGZ);

    // --- Compute elevation (pitch) angle ---
    // Subtract calibration offset to get angle relative to equilibrium
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) - pitchOffset;

    // --- Compute roll and yaw from potentiometers ---
    float currentRoll = (analogRead(A0) * ADC_SCALE) - rollOffset;
    float currentYaw  = (analogRead(A1) * ADC_SCALE) - yawOffset;

    // --- Gyroscope Y-axis rate (elevation rate proxy) ---
    // Sign inversion and scaling combined into a single operation
    float gy = (rawGY - gyOffset) * -0.05f;

    // --- Roll velocity estimation with low-pass filter and deadband ---
    float dRoll = currentRoll - lastRoll;
    if (dRoll > DEADBAND || dRoll < -DEADBAND) {
      // Outside deadband: update filter with new derivative estimate
      filteredRollSpeed = LP_ALPHA * (dRoll * dt_inv) + LP_ALPHA_INV * filteredRollSpeed;
    } else {
      // Inside deadband: passive decay (avoids integrating noise)
      filteredRollSpeed *= 0.9f;
    }
    lastRoll = currentRoll;

    // --- Yaw velocity estimation with low-pass filter and deadband ---
    float dYaw = currentYaw - lastYaw;
    if (dYaw > DEADBAND || dYaw < -DEADBAND) {
      filteredYawSpeed = LP_ALPHA * (dYaw * dt_inv) + LP_ALPHA_INV * filteredYawSpeed;
    } else {
      filteredYawSpeed *= 0.9f;
    }
    lastYaw = currentYaw;

    // --- Transmit state vector to LabVIEW ---
    // Format: e, theta, psi, e_dot, theta_dot, psi_dot
    // Signs adjusted to match model convention (positive = positive direction)
    Serial.print(-pitch, 3);             Serial.print(',');   // e
    Serial.print(currentRoll, 3);        Serial.print(',');   // theta
    Serial.print(-currentYaw, 3);        Serial.print(',');   // psi
    Serial.print(gy, 3);                 Serial.print(',');   // e_dot
    Serial.print(filteredRollSpeed, 3);  Serial.print(',');   // theta_dot
    Serial.print(-filteredYawSpeed, 3);  Serial.print(',');   // psi_dot
    Serial.print(micros() - loopStart);  Serial.print('\n');  // loop time (µs)
  }

  // -------------------------------------------------------------------------
  // Receive ESC commands from LabVIEW
  // -------------------------------------------------------------------------
  // Format: "<esc1_microseconds>,<esc2_microseconds>\n"
  // Values are hardware-clamped to [ESC_MIN, ESC_MAX] for safety
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    if (commaIndex != -1) {
      int val1 = constrain(data.substring(0, commaIndex).toInt(), ESC_MIN, ESC_MAX);
      int val2 = constrain(data.substring(commaIndex + 1).toInt(), ESC_MIN, ESC_MAX);
      esc1.writeMicroseconds(val1);
      esc2.writeMicroseconds(val2);
    }
  }
}
