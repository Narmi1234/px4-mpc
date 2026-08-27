# Gate D attempt 04 — infrastructure FAIL before transition

```text
ULog: /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-27/16_35_53.ulg
size: 45,341,465 bytes
sha256: 5fdf5904ef2d6aa216e4940953d2e22e149018dd96e09454c3e61cf25de85fd2
result: FAIL (odometry_stale in mc_accelerate)
```

No transition command was sent (`front_ack=False`, `back_ack=False`). At the
abort the aircraft was still in benign MC acceleration: CAS `6.024 m/s`,
altitude error `0.163 m`, tilt `1.73 deg`, vertical speed `0.086 m/s`, pusher
`0.239`, and zero solver failures.

ROS measured a one-off state wall/PX4 gap of `0.365/0.329 s`. The PX4 ULog had
no logger dropouts and `vehicle_local_position` remained continuous with a
median interval `0.008 s`, p99 `0.016 s`, and maximum `0.032 s`. This was a DDS
delivery gap, not an estimator or plant failure.

A bounded `0.45 s` gap allowance is now used only during `mc_accelerate`,
matching the previously validated pretransition allowance. As soon as Gate D
enters `front_transition`, the strict `0.30 s` wall and `0.20 s` PX4-age limits
remain active.
