# Gate D Attempt 07 — post-FW pitch-authority failure

Date: 2026-08-27

Result: FAIL (`vertical_speed_limit`). The `0.40` pusher correction removed
the Attempt 06 entry overspeed and PX4 completed the full state sequence, but
the NMPC pitch-rate governor did not have enough authority immediately after
lift-motor shutdown.

```text
transition_state_history=[3, 1, 4, 2, 3]
front transition duration=2.016 s
FW duration before recovery=0.768 s
fw_entry_airspeed=10.586 m/s
active NMPC maximum ground speed=11.248 m/s
active NMPC maximum altitude error=1.297 m
active NMPC maximum vertical speed=3.096 m/s
active NMPC maximum tilt=17.26 deg
pusher_peak=0.4000
solver_failures=1
```

The ULog shows lift-motor shutdown and actual FW entry. During the first
`0.5 s` of FW, actual pitch rate reached approximately `-0.64 rad/s`, while
the guarded setpoint could rise only toward `+0.10 rad/s`; the elevator then
saturated. Recovery correctly requested back transition, but its former zero
rate command allowed the altitude excursion to grow to `29.696 m` before MC
recovery. The ROS maxima excluded most of that recovery interval, which is
why the final ROS status reported only `1.297 m`.

Stock transition ULogs show pitch-rate setpoints around `0.44..0.55 rad/s`
during the same transient. The next configuration keeps the gentle front
transition governor but uses a `0.65 rad/s`, `1.50 rad/s^2`, gain-3 pitch
envelope in confirmed FW/back transition, plus active pitch leveling during
abort recovery.

Post-fix offline closed loop:

```text
gate_d_state=complete
final_vtol_state=3
duration=53.05 s
solver_failures=0
maximum_speed=12.318 m/s
maximum_altitude_error=1.023 m
maximum_tilt=7.50 deg
maximum_pusher=0.400
final_speed=0.071 m/s
transition_gate_d_offline=PASS
```

ULog (kept outside git):

```text
/home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-27/17_32_28.ulg
sha256=9639f1d4911444ef6995ff2f96ef1033160fdb7df5577b00f26326d88f38e08f
size=37 MiB
```
