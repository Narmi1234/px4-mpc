# Gate D Attempt 09 — FW pusher slew and hard OCP constraint

Date: 2026-08-28

Result: FAIL (`vertical_speed_limit`). PX4 completed `3 -> 1 -> 4 -> 2 -> 3`,
but the intended confirmed-FW pusher boost could not reach its command before
the pitch/vertical-speed abort.

```text
front transition duration=2.060 s
FW duration before recovery=0.776 s
fw_entry_airspeed=10.003 m/s
peak calibrated airspeed=13.066 m/s
peak ground speed=13.072 m/s
altitude loss through recovery=6.780 m
maximum tilt=21.49 deg
pusher peak=0.4000
solver_failures=3
```

ULog thrust-setpoint interpolation proves that PX4 FW output followed the
Offboard command, but that command ramped only from `0.086` to `0.254` during
FW because the node retained a `0.33/s` slew. The aircraft reached recovery
before the requested `0.60` was achievable. As pitch crossed the Gate D OCP's
former hard `15/20 deg` attitude bounds, three consecutive solves also became
infeasible, removing useful feedback at exactly the wrong time.

Next configuration uses `2.0/s` pusher rise and reduction only in confirmed
`fw_hold`, preserving the `0.40` front-transition ceiling. Gate D's internal
OCP attitude domain is widened to `+/-35 deg` so it remains solvable during
recovery, while the independent live safety watchdog remains at `20 deg`.

Post-fix offline closed loop:

```text
gate_d_state=complete
final_vtol_state=3
duration=58.95 s
solver_failures=0
maximum_speed=15.168 m/s
maximum_altitude_error=1.401 m
maximum_tilt=8.03 deg
maximum_pusher=0.600
final_speed=0.098 m/s
transition_gate_d_offline=PASS
```

ULog (kept outside git):

```text
/home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-27/18_26_01.ulg
sha256=c52ef4d16cf9e397ecc1a50cb367725b9a73b5395e68a9d335374106c78fb003
size=54 MiB
```
