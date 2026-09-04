# Robust allocation L4a-v2 — accepted PASS

Date: 2026-09-04

Raw ULog: `/home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-04/06_11_49.ulg`

Preserved artifact: `accepted/robust_allocation_l4a_v2_pass_2026-09-04.ulg.zst`

SHA-256: `063978648ec0318585600eac5b70b6e16367ab6caca642b1cd190a125fe116fc`

Result: **PASS**, `abort_reason=allocation_l4a_test_timeout`.

The gate completed 108.00 s with solver status 0 and zero solver failures.
Maxima were 12.207 m/s groundspeed, 12.028 m/s CAS, 0.176 m altitude error,
0.387 m controller-frame cross-track, pusher 0.321 and elevator feed-forward
0.250. Minimum applied lift weight was 0.234, minimum MC roll/pitch torque
weight 0.092, and MC yaw weight remained 1.000. All allocation weights
returned to 1.0 before Position fallback.

Scientific interpretation: the corrected identified lateral model and PX4
FW-rate execution patch stabilized a flight in which aerodynamic surfaces
carried about 91% of roll/pitch torque authority. This validates L4a only;
direct MC yaw remained active and lift motors were not stopped.

The accepted run is nearly straight and therefore is a closed-loop validation
run, not a new identification dataset. The older excited L3c/L4a datasets
remain the source of the frozen roll/course coefficients.
