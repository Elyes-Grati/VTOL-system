#include <Servo.h>
#include <Wire.h>
#include <math.h>

#define ACCEL_ADDR  0x53
#define GYRO_ADDR   0x68

Servo esc1, esc2;

const int ESC1_PIN = 9, ESC2_PIN = 10;
const int ESC_MIN = 1000, ESC_MAX = 1700;

const float GYRO_SCALE    = 1.0 / 14.375;  // Pre-inverted: multiply is faster than divide
const float LP_ALPHA      = 0.15f;
const float LP_ALPHA_INV  = 0.8f;          // Pre-computed 1.0 - LP_ALPHA
const float DEADBAND      = 0.002f;
const float ADC_SCALE     = 4.7123f / 1024.0f; // Pre-computed ADC conversion factor

float pitchOffset = 0, rollOffset = 2.91f, yawOffset = 0;
float gxOffset = 0, gyOffset = 0;            // gzOffset removed (never used)

// Inline I2C write — eliminates function call overhead on hot path
inline void writeTo(uint8_t device, uint8_t address, uint8_t val) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}

void calibrate() {
  float sumP = 0, sumGX = 0, sumGY = 0;
  float sumR = 0, sumY = 0;

  for (int i = 0; i < 100; i++) {
    float ax, ay, az, gx, gy, gz;
    readAccel(&ax, &ay, &az);
    readGyro(&gx, &gy, &gz);

    sumP  += atan2f(-ax, sqrtf(ay * ay + az * az)); // sqrtf/atan2f: float versions, faster on AVR
    sumR  += analogRead(A0) * ADC_SCALE;
    sumY  += analogRead(A1) * ADC_SCALE;
    sumGX += gx;
    sumGY += gy;
  }

  pitchOffset = sumP  * 0.01f;  // Multiply by 1/100 instead of divide
  rollOffset  = sumR  * 0.01f;
  yawOffset   = sumY  * 0.01f;
  gxOffset    = sumGX * 0.01f;
  gyOffset    = sumGY * 0.01f;
}

void setup() {
  Wire.begin();
  Serial.begin(230400);

  writeTo(ACCEL_ADDR, 0x2D, 8);
  writeTo(0x0D,       0x09, 0x01);
  writeTo(GYRO_ADDR,  0x3E, 0x01);
  writeTo(GYRO_ADDR,  0x16, 0x18);

  delay(500);
  calibrate();

  esc1.attach(ESC1_PIN);
  esc2.attach(ESC2_PIN);

  esc1.writeMicroseconds(ESC_MAX);
  esc2.writeMicroseconds(ESC_MAX);
  delay(3000);
  Serial.println("test1");

  esc1.writeMicroseconds(ESC_MIN);
  esc2.writeMicroseconds(ESC_MIN);
  delay(3000);
  Serial.println("test2");
}

void loop() {
  unsigned long loopStart = micros();
  static unsigned long lastMicros = 0;
  static float lastRoll = 0, filteredRollSpeed = 0;
  static float lastYaw  = 0, filteredYawSpeed  = 0;

  unsigned long currentMicros = micros();

  if (currentMicros - lastMicros >= 10000) {
    float dt_inv = 1000000.0f / (float)(currentMicros - lastMicros); // Pre-invert dt
    lastMicros = currentMicros;

    float ax, ay, az, rawGX, rawGY, rawGZ;
    readAccel(&ax, &ay, &az);
    readGyro(&rawGX, &rawGY, &rawGZ);

    float pitch       = atan2f(-ax, sqrtf(ay * ay + az * az)) - pitchOffset;
    float currentRoll = (analogRead(A0) * ADC_SCALE) - rollOffset;
    float currentYaw  = (analogRead(A1) * ADC_SCALE) - yawOffset;
    float gy          = (rawGY - gyOffset) * -0.05f;  // Fold sign+scale into one op

    // Roll speed
    float dRoll = currentRoll - lastRoll;
    if (dRoll > DEADBAND || dRoll < -DEADBAND) {  // Avoid fabsf() call
      filteredRollSpeed = LP_ALPHA * (dRoll * dt_inv) + LP_ALPHA_INV * filteredRollSpeed;
    } else {
      filteredRollSpeed *= 0.9f;
    }
    lastRoll = currentRoll;

    // Yaw speed
    float dYaw = currentYaw - lastYaw;
    if (dYaw > DEADBAND || dYaw < -DEADBAND) {
      filteredYawSpeed = LP_ALPHA * (dYaw * dt_inv) + LP_ALPHA_INV * filteredYawSpeed;
    } else {
      filteredYawSpeed *= 0.9f;
    }
    lastYaw = currentYaw;

    // Output — same format, same values
    Serial.print(-pitch, 3);            Serial.print(',');
    Serial.print(currentRoll, 3);       Serial.print(',');
    Serial.print(-currentYaw, 3);       Serial.print(',');
    Serial.print(gy, 3);                Serial.print(',');
    Serial.print(filteredRollSpeed, 3); Serial.print(',');
    Serial.print(-filteredYawSpeed, 3); Serial.print(',');
     Serial.print( micros() - loopStart);  Serial.print('\n');
  }

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