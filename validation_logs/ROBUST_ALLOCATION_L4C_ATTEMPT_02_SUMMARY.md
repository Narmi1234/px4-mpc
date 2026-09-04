# Robust allocation L4c-v2 attempt 02 — low-CAS motor-off, gate FAIL

Date: 2026-09-04

Raw ULog: `/home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-04/19_45_22.ulg`

Preserved artifact: `accepted/robust_allocation_l4c_v2_low_cas_fail_2026-09-04.ulg.zst`

SHA-256: `59f107f497a6b952d76fde514ae07d8fcbae45d32bc94667330cedff97e2b0b1`

Outcome: `altitude_error` after 52.25 s; not a gate PASS.

- solver failures: 0
- maximum groundspeed: 12.571 m/s
- maximum calibrated airspeed: 9.922 m/s
- minimum lift allocation: 0.000
- minimum MC roll/pitch and yaw weights: 0.050 / 0.050
- continuous motor-off proof: 4.58 s
- maximum altitude error: 1.193 m

The key failure was scheduling motor-off from groundspeed while the model used
zero wind. L4c-v3 estimates longitudinal wind from groundspeed minus CAS,
adds a physical CAS-based lift-allocation floor and requires CAS >= 12 m/s
before zero lift allocation. No safety guard was relaxed.
