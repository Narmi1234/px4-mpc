# Gate D Attempt 08 — insufficient post-transition airspeed

Date: 2026-08-27

Result: FAIL (`vertical_speed_limit`). PX4 completed `3 -> 1 -> 4 -> 2 -> 3`.
The stronger pitch governor and active recovery substantially reduced the
previous altitude loss, but the 12 m/s FW corridor did not provide enough
aerodynamic control authority immediately after lift-motor shutdown.

```text
front transition duration=2.028 s
FW duration before recovery=0.756 s
fw_entry_airspeed=10.333 m/s
peak calibrated airspeed=13.453 m/s
peak ground speed=13.492 m/s
altitude loss through recovery=5.447 m
maximum tilt=17.58 deg
pusher peak=0.4000
```

At FW entry the pitch governor reached `0.25`, `0.49`, then `0.65 rad/s`, but
airspeed was only `10.55`, `10.89`, then `11.65 m/s`; actual pitch still fell
to `-17.4 deg` before aerodynamic recovery. Stock PX4 logs starting near the
same `10.2..10.6 m/s` FW-entry airspeed accelerate to `16.5..17.3 m/s` within
`0.6 s`, limiting vertical speed to roughly `1.4..2.1 m/s` and recovering
pitch.

Next configuration therefore keeps the validated front-transition pusher
ceiling `0.40`, but after confirmed FW state allows `0.60` until CAS reaches
`14 m/s`, tracks a `15 m/s` FW reference, and uses an `18.5 m/s` hard safety
limit. This is a phase-specific control-authority requirement, not a blanket
pusher increase during MC/front transition.

Post-fix offline closed loop:

```text
gate_d_state=complete
final_vtol_state=3
duration=58.95 s
solver_failures=0
maximum_speed=15.170 m/s
maximum_altitude_error=1.404 m
maximum_tilt=8.03 deg
maximum_pusher=0.492
final_speed=0.098 m/s
transition_gate_d_offline=PASS
```

ULog (kept outside git):

```text
/home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-27/18_11_13.ulg
sha256=16bb32e2e84de29043293a4cdf1b011fcc91c29c6e0374c727afd549f68e9a04
size=60 MiB
```
