# Hardware Setup Guide

## Wiring Diagram

```
Arduino Uno
───────────────────────────────────────────────────────────
Pin A4  (SDA) ──────┬───── ADXL345 SDA
                    └───── ITG-3205 SDA
Pin A5  (SCL) ──────┬───── ADXL345 SCL
                    └───── ITG-3205 SCL
Pin A0  ────────────────── Potentiometer 1 (Roll/θ) wiper
Pin A1  ────────────────── Potentiometer 2 (Yaw/Ψ) wiper
Pin  9  (PWM)  ──────────── ESC 1 signal wire (Motor M1)
Pin 10  (PWM)  ──────────── ESC 2 signal wire (Motor M2)
5V   ───────────────────── ADXL345 VCC, ITG-3205 VCC, Pot VCC
GND  ───────────────────── All ground commons
USB  ───────────────────── PC (LabVIEW serial communication)

Separate power supply (7.4V–11.1V LiPo or bench supply):
  (+) ────────────────────── ESC 1 (+), ESC 2 (+)
  (−) ────────────────────── ESC 1 (−), ESC 2 (−), Arduino GND
```

## Potentiometer Mounting

The two potentiometers must be rigidly coupled to the mechanical pivot joints:

- **Potentiometer 1 (A0):** Mounted at the roll axis. The wiper rotates with the
  wing arm. At equilibrium (θ = 0), the wiper should be near midpoint.
- **Potentiometer 2 (A1):** Mounted at the yaw axis. The wiper rotates with the
  yaw pivot.

The ADC mapping assumes 0–1023 → 0 to 3π/2 radians. Verify the mechanical
stops do not exceed this range to avoid ADC clipping.

## IMU Orientation

The ADXL345 must be mounted such that:
- The **Z-axis** points upward at hover (parallel to gravity, opposite direction)
- The **X-axis** points along the arm toward the motors

The elevation angle is computed as:
```
e = atan2(-ax, sqrt(ay² + az²))
```
If the physical Z-axis is inverted, negate `az` in the firmware.

## ESC Wiring and Safety

**Warning:** Brushless motors and ESCs carry sufficient current to cause burns
or fire if improperly wired. Follow these precautions:

1. Always disconnect the motor power supply when flashing firmware
2. Remove propellers when running the ESC calibration sequence for the first time
3. The ESC signal wire (PWM) is isolated from the motor power rail.
   Only the signal and ground wires connect to the Arduino
4. Ensure the Arduino GND is common with the ESC negative rail

## First Power-On Checklist

- [ ] Arduino connected to PC via USB, LabVIEW COM port identified
- [ ] `export_data.csv` present in LabVIEW VI directory
- [ ] Motor power supply disconnected
- [ ] Propellers removed
- [ ] Open Arduino Serial Monitor at 230400 baud, verify sensor readings
- [ ] Close Serial Monitor (LabVIEW will use the port)
- [ ] Run LabVIEW VI, observe ESC calibration messages in front panel log
- [ ] Connect motor power supply
- [ ] Attach propellers
- [ ] Enable control loop from LabVIEW front panel
