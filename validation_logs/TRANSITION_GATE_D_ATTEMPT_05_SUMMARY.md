# Gate D attempt 05 — PX4 Offboard-rate transition blocker

```text
ULog: /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-27/16_47_13.ulg
size: 38,151,883 bytes
sha256: 908c030a9962e0f2d9a5e0f74840654850d26507f0e26fbc453db1fe3605762d
result: FAIL (horizontal_speed_limit; PX4 never confirmed FW)
```

The faster front profile worked as intended. CAS crossed `8/10/11/12/13/14`
m/s at approximately `0.61/1.31/1.61/1.71/2.11/2.32 s` after the front
request. Active maxima remained bounded: tilt `3.88 deg`, altitude error
`1.024 m`, vertical speed `1.352 m/s`, pusher `0.520`, and zero solver
failures. Recovery returned to MC.

## Confirmed PX4 root cause

The ULog state sequence was only `MC -> TRANSITION_TO_FW -> MC`, despite CAS
exceeding `VT_ARSP_TRANS=10 m/s` after the configured
`VT_TRANS_MIN_TM=2 s`. In `vtol_att_control_main.cpp`, transition-state update
is called only when a new MC or FW virtual attitude setpoint arrives. Gate D
uses Offboard body-rate mode, so neither attitude-setpoint stream updates.
Consequently `VtolType::_time_since_trans_start` stays zero, Standard VTOL's
transition weights do not update, and the minimum-time completion condition
can never become true.

No further NMPC pusher, speed, gain, or watchdog tuning can fix this blocker.
The next step requires an explicitly approved PX4 firmware change scoped to
Standard VTOL Offboard-rate transition timing/weights, followed by removal of
the NMPC-side explicit lift blend to prevent double blending, a PX4 rebuild,
offline regression, and a fresh Gate D attempt.
