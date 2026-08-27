# Gate D Attempt 06 — PX4 transition patch validation

Date: 2026-08-27

Result: ROS gate FAIL (`horizontal_speed_limit`), but the PX4 firmware patch
successfully completed the full VTOL state cycle.

```text
transition_state_history=[3, 1, 4, 2, 3]
front transition duration=2.016 s
FW duration before recovery=0.060 s
fw_entry_airspeed=13.271 m/s
peak_calibrated_airspeed=14.635 m/s
peak_ground_speed=14.665 m/s
altitude_loss during transition/recovery=6.054 m
maximum_tilt=11.49 deg
pusher_peak=0.5139
lift_motor_range=[0.0087, 0.5398]
solver_failures=0
```

The pusher did not turn off around 9 m/s. ULog interpolation shows actual
pusher approximately `0.46` at ground speed `9.62 m/s`, `0.49` at
`12.10 m/s`, and `0.44` at FW entry. The visible motor shutdown was the
expected PX4 lift-motor blend. Gate D then crossed its 14 m/s safety limit at
FW entry and immediately requested back transition.

Root cause: the live node forced a minimum `0.60` pusher command throughout
front transition, while its overspeed governor had a `0.75 m/s` deadband and
could remove only `0.10/s`. The next configuration uses a `0.40` pusher
ceiling/floor only while speed is within `0.25 m/s` of reference and removes
pusher at `0.33/s` above that corridor.

Post-fix offline closed loop:

```text
gate_d_state=complete
final_vtol_state=3
duration=53.05 s
solver_failures=0
maximum_speed=12.307 m/s
maximum_altitude_error=1.167 m
maximum_tilt=6.72 deg
maximum_pusher=0.400
final_speed=0.071 m/s
transition_gate_d_offline=PASS
```

ULog (kept outside git):

```text
/home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-27/17_16_16.ulg
sha256=3e26041f4b03957a11e239f0d7400269a0e9bb2c59a9ccf3b81dc88fe4722154
size=50 MiB
```
