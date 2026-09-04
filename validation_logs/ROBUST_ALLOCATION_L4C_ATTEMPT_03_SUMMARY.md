# Robust allocation L4c-v3 attempt 03 — low-CAS oscillation, gate FAIL

Date: 2026-09-04

Raw ULog: `/home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-04/20_07_09.ulg`

Preserved artifact: `accepted/robust_allocation_l4c_v3_low_cas_oscillation_fail_2026-09-04.ulg.zst`

SHA-256: `ba1fd7d51a8818404257cb67a8270234154d78c43f2e2cf8ae733f8c2dca591c`

Outcome: `vertical_speed_emergency_limit` after 7.40 s; not a gate PASS.

- solver failures: 0
- maximum groundspeed: 1.713 m/s
- maximum calibrated airspeed: 2.155 m/s
- minimum lift allocation: 0.910
- maximum altitude error: 0.545 m
- ULog maximum absolute vertical speed during Offboard: 1.273 m/s
- lift motors never reached the motor-off condition

The first CAS interlock was linear from zero to 12 m/s. The simulated pitot
varied approximately from -2.5 to +2.2 m/s in low-speed flight, causing the
effective allocation constraint to move despite insufficient aerodynamic
information. The corrected gate retains full MC lift through CAS=4 m/s,
smoothly unloads from 4 to 12 m/s, and blends in the longitudinal wind
estimate only above 4 m/s. No flight-safety threshold was relaxed.
