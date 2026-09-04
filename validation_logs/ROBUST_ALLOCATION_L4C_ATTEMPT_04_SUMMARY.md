# Robust allocation L4c-v3 attempt 04 — vertical handover, gate FAIL

Date: 2026-09-04

Raw ULog: `/home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-04/20_32_01.ulg`

Preserved artifact: `accepted/robust_allocation_l4c_v3_full_lift_vertical_fail_2026-09-04.ulg.zst`

SHA-256: `dcfa9b29210e9c02945f0a7e4862f694ae53c6d5b36e1ea75e32b64ed8d6ef64`

Outcome: `vertical_speed_limit` after 5.02 s; not a gate PASS.

- solver failures: 0
- maximum groundspeed: 1.026 m/s
- maximum calibrated airspeed: 1.959 m/s
- minimum lift allocation: 1.000
- maximum altitude error: 0.681 m
- ULog maximum absolute vertical speed during Offboard: 1.003 m/s
- lift motors were fully allocated throughout the attempt
- vertical speed immediately before Offboard was approximately 0.11 m/s

The CAS deadband worked, so this was not premature lift transfer. The failure
was isolated to the vertical Offboard handover: low-speed OCP collective and
the external vertical correction fought before meaningful aerodynamic speed.
The corrected controller requires `|vz| <= 0.08 m/s` at start, retains the
flight-proven hover collective law through CAS=4 m/s, and blends into NMPC
collective over CAS=4–6 m/s. No guard threshold was relaxed.
