# Robust allocation L4c attempt 01 — motor-off reached, gate FAIL

Date: 2026-09-04

Raw ULog: `/home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-04/16_48_58.ulg`

Preserved artifact: `accepted/robust_allocation_l4c_12mps_altitude_fail_2026-09-04.ulg.zst`

SHA-256: `753542a7cd1f88ae169317acb1190fd9fdcb0dd100b25644231d3bc7c4369f19`

Outcome: `altitude_error` after 56.05 s; this is not a complete gate PASS.

- solver status/failures: 0 / 0
- maximum groundspeed: 12.479 m/s
- maximum CAS: 11.117 m/s
- minimum lift allocation: 0.000
- minimum MC roll/pitch and yaw weights: 0.050 / 0.050
- continuous motor-off proof: 8.36 s
- maximum altitude error: 1.193 m
- lift-motor actuator mean during motor-off: approximately 0.01

The terminal trace and ULog show a growing pitch/vertical oscillation while
the wing-borne allocation is active. The independent
`standard_vtol_run_03_12ms.ulg` gives a 4.10 degree median stable FRD pitch at
CAS 10–12.5 m/s and |vz| <= 0.3 m/s; L4c attempt 01 referenced only 1.36
degrees at full speed. L4c-v2 corrects that trim without relaxing a guard.
