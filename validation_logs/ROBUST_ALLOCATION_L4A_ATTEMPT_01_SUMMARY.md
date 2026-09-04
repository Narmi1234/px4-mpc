# Robust allocation L4a — attempt 01

Date: 2026-09-03

ULog: `/home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-09-03/17_56_14.ulg`

Result: **FAIL**, `abort_reason=l4a_cross_track_limit`.

Measured maxima: forward speed 12.657 m/s, CAS 12.696 m/s, cross-track
2.473 m, altitude error 1.071 m, minimum lift weight 0.237, minimum MC
roll/pitch weight 0.094, MC yaw weight 1.000, and zero solver failures.

The failure isolated a model/execution mismatch rather than an optimization
failure. The old OCP retained fixed full-MC roll-rate dynamics after PX4 had
transferred most roll/pitch torque to the surfaces. ULog identification gives
`p_dot = -a_p p + b_p V^2 delta_a + c`, with the two bounding runs yielding
`a_p=0.34..0.67` and `b_p=0.041..0.055`. Course replay validates
`chi_dot = k g tan(phi)/V`, `k=0.94..0.96`.

Corrective implementation: nominal `a_p=0.50`, `b_p=0.0475`, `k=0.95`,
explicit per-axis allocation parameters in the OCP, coordinated yaw-rate
output, PX4 FW P/FF airspeed scaling, and PX4 commit `b722b3e6bb` retaining
FW rate-controller state only during a fresh guarded external allocation.

Only one L4a-v2 go/no-go repeat is authorized by the runbook.
