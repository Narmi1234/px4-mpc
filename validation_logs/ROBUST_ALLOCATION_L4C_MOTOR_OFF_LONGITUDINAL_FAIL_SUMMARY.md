# Robust allocation L4c — motor-off proof, longitudinal gate FAIL

Date: 2026-09-04

Raw ULog: `/home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-04/20_34_57.ulg`

Preserved artifact: `accepted/robust_allocation_l4c_motor_off_longitudinal_fail_2026-09-04.ulg.zst`

SHA-256: `47a1e47ba9b5d1cd827e51c78e11321cad90ce0e2040bc9bef4b7df9b1236ee9`

Outcome: `vertical_speed_limit`; motor-off proof succeeded, overall gate FAIL.

- ULog Offboard interval: 51.968 s
- node-reported Offboard duration: 56.50 s
- solver failures: 0
- maximum calibrated airspeed: 13.654 m/s
- minimum applied allocation: 0.000
- minimum MC roll/pitch and yaw weights: 0.050 / 0.050
- continuous motor-off proof: 7.30 s
- mean lift-motor output minimum: 0.0093
- maximum absolute altitude displacement in ULog: 0.745 m
- maximum absolute vertical speed in ULog: 1.026 m/s
- motor-off pitch range: -1.76 to +5.01 deg
- motor-off NED vertical-speed range: -1.02 to +0.69 m/s

The vehicle had not entered the planned braking segment. At abort, the
accelerating profile reference was approximately 12.74 m/s. The visible
deceleration occurred after the guard requested Position mode. The measured
motor-off oscillation therefore identifies a wing-borne longitudinal model
and control mismatch, not a ROS, solver, allocation-channel, or planned-brake
failure.

Do not repeat L4c or enable a full VTOL-state transition with the current
model. First identify/revalidate the aerodynamic lift-drag-pitch subsystem on
this motor-off interval, demonstrate stable offline closed-loop recovery, and
implement asymmetric allocation recovery (slow unloading, faster MC motor
re-engagement).
