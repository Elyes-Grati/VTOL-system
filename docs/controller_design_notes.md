# Controller Design Notes

## LQR-I Design Rationale

### Why Integral Action on e and Ψ only?

The system has 3 position outputs: elevation (e), roll (θ), and yaw (Ψ).
Integral action is added only to e and Ψ for the following reasons:

**Elevation (e):** The hover equilibrium requires a constant thrust offset u₀ = mg.
Any small modeling error in mass or gravity produces a constant force imbalance that
a pure LQR cannot reject. The integral on e corrects this automatically.

**Yaw (Ψ):** Motor-to-motor thrust asymmetries (manufacturing tolerances, different
propeller wear) produce a constant yaw torque. The integral on Ψ eliminates this.

**Roll (θ):** Roll is not integrated because:
1. The rig is physically symmetric — no expected constant roll bias
2. Adding an integrator on θ risks integrator windup during large transients
3. The high Q weight on θ (300) already provides tight regulation

### Why ε = 0.2?

The coupling coefficient ε represents the fraction of roll torque that converts
into a lateral body force. In the 3-DOF rig, this appears in:
- The elevation equation: l(u·cosθ + ε·v·sinθ) — roll torque perturbs elevation
- The yaw equation: l(u·sinθ + ε·v·cosθ)·cose — roll torque drives yaw

ε = 0.2 is a physical property of the thruster geometry (angle α between the
thrust vector and the body vertical axis). Setting ε = 0 would mean perfectly
aligned thrusters with no coupling — physically unrealistic for this rig.

### Gain Matrix Interpretation

```
K = [ 0.9287    0        0       0.8366   0       0       -0.3162    0      ]
    [ 0         1.7954   1.3396  0        0.6171  2.5070   0        -0.2582 ]
```

**Row 1 (collective thrust u = F1 + F2):**
- K[1,1] = 0.9287: Elevation error e drives collective thrust (main lift loop)
- K[1,4] = 0.8366: Elevation rate ė provides velocity damping
- K[1,7] = -0.3162: Negative gain on ∫e — integral action (note: sign convention
  means positive ∫e → reduce thrust, which is correct for error rejection)

**Row 2 (differential thrust v = F1 - F2):**
- K[2,2] = 1.7954: Roll error θ drives differential thrust (roll stabilization)
- K[2,3] = 1.3396: Yaw error Ψ drives differential thrust (yaw control)
- K[2,5] = 0.6171: Roll rate θ̇ damping
- K[2,6] = 2.5070: Yaw rate Ψ̇ damping — largest gain, reflecting the
  significant inertial resistance of the yaw axis (J_ψ = 0.23 kg·m²)
- K[2,8] = -0.2582: Integral action on yaw

The zero entries reflect the decoupled structure of the system after
linearization: the u input does not directly affect θ or Ψ in the linear model,
and v does not directly affect e.

## Serial Protocol Details

### Baud Rate Selection: 230400

At 100 Hz loop rate, each transmitted message is approximately:
```
"-0.123,1.234,-0.567,0.089,0.012,-0.034,145\n"  ≈ 44 characters
```
At 230400 baud with 10 bits/character (8N1): 23040 chars/second capacity.
At 100 Hz: 44 × 100 = 4400 chars/second required — well within budget.

At 9600 baud (960 chars/second), the serial buffer would overflow at 100 Hz.
115200 baud would be marginal. 230400 provides comfortable headroom.

### Loop Timing

The last field in each TX message is the loop execution time in microseconds.
This allows monitoring of computational budget consumption from LabVIEW.
Nominal loop time is well under 1000 µs, leaving 9000 µs idle per 10 ms period.

## ESC Calibration Procedure

The ESC calibration sequence on startup (MAX then MIN pulse) is required by
most brushless ESC firmware (BLHeli, SimonK, etc.) to learn the throttle range.

If the ESC has already been calibrated to [1000, 1700] µs, this sequence
acts as a verification pass. If the ESC firmware is reset or a new ESC
is installed, this sequence must complete without interruption.

**Important:** Motors must NOT be armed (propellers attached) during the
first power-on calibration with a new ESC.
