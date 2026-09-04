# Robust allocation L4b — accepted PASS

Date: 2026-09-04

Raw ULog: `/home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-04/06_38_42.ulg`

Preserved artifact: `accepted/robust_allocation_l4b_pass_2026-09-04.ulg.zst`

SHA-256: `f8d539491f66cc4b312293b63830b61b040c2f3a76226d64f40df5b3526e5512`

The node completed the complete 108.05 s L4b profile with
`allocation_l4b_test_timeout`, solver status 0, zero solver failures, valid
PX4 allocation feedback and automatic Position fallback.

- maximum forward speed: 12.260 m/s
- maximum calibrated airspeed: 12.274 m/s
- maximum altitude error: 0.586 m
- maximum cross-track error: 0.860 m
- maximum pusher command: 0.315
- minimum lift weight: 0.250
- minimum MC roll/pitch weight: 0.109
- minimum MC yaw weight: 0.109

The approximately 89% reduction of direct MC attitude torque was exercised
during a smooth lateral out-and-back reference. This validates L4b only. It
does not claim lift-motor shutdown or a complete VTOL mode transition.

The terminal initially printed `ROBUST_ALLOCATION_L4B=FAIL` because its old
string check accepted only values beginning with `0.0`; the node's actual,
predeclared acceptance threshold was 0.13. The runner now parses and compares
the measured values numerically.
