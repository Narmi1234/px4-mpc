# Gate D attempt 01 — diagnosed FAIL

```text
ULog: /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-26/18_03_05.ulg
size: 93,231,597 bytes
sha256: b1a2b6e22f80d6257c208c856ae00b9eb960aa196696c7b59ed83679436d10df
result: FAIL (altitude_error)
```

The guarded recovery worked: PX4 accepted both transition commands, returned
to MC, and only then entered Position mode. The aircraft never entered FW;
the ULog state history for the attempt was `MC -> TRANSITION_TO_FW -> MC`.

ROS maxima before abort:

```text
offboard duration:    31.11 s
forward speed:        10.473 m/s
calibrated airspeed:  10.320 m/s
altitude error:       2.343 m
tilt:                 8.85 deg
cross-track:          0.562 m
commanded pusher:     0.269
```

## Root cause

The altitude limit was exceeded upward, not downward. During the front
transition the outgoing multicopter collective stayed at approximately
`0.48..0.52` even after CAS crossed the `8 m/s` blend threshold. Gate D had
modeled `effective collective = mc_weight * raw collective`, but its live
output limiter kept sending hover-range raw collective and assumed PX4 would
apply that weight. In this Offboard body-rate path, the logged outgoing
`vehicle_thrust_setpoint[0].xyz[2]` remained equal to the unweighted virtual
MC thrust. Wing lift and nearly full lift-motor thrust therefore produced the
climb.

The eight reported solver failures occurred after the abort while PX4 was
braking in MC Position mode and pitch moved outside the NMPC OCP corridor.
They did not trigger the abort. The node continued shadow-solving after its
result was already final, so these were misleading post-abort failures.

## Correction before attempt 02

- Gate D now explicitly multiplies outgoing collective by its measured
  PX4-equivalent lift-weight schedule.
- The transition limiter permits collective below the hover floor and follows
  the smooth blend with a bounded `0.30 command/s` slew.
- Abort recovery bypasses NMPC and sends deterministic hover collective,
  pusher-to-zero and zero body rates.
- No NMPC solve is attempted after Gate D reaches `failed` or `complete`, so
  post-result PX4 braking cannot alter solver-failure metrics.
- The external-pusher PX4 branch is unchanged.

The corrected full offline rehearsal passes in `58.95 s`, with peak speed
`11.964 m/s`, maximum altitude error `0.903 m`, tilt `7.77 deg`, zero solver
failures, final MC speed `0.074 m/s`, and final pusher `0.0001`.
