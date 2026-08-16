#!/usr/bin/env python3
"""Offline bounds check for the first PX4 MC pusher-assist profile."""

import numpy as np

from px4_mpc.controllers.standard_vtol_output import expected_px4_pusher_assist
from px4_mpc.models.mc_forward_profile import McForwardProfile


def main() -> None:
    profile = McForwardProfile(
        target_speed=3.0,
        acceleration=1.5,
        hold_seconds=1.0,
        start_delay_seconds=2.0,
    )
    times = np.linspace(0.0, 13.0, 1301)
    samples = [profile.sample(time) for time in times]
    speeds = np.array([sample.speed for sample in samples])
    accelerations = np.array([sample.acceleration for sample in samples])
    pusher_estimates = np.array(
        [expected_px4_pusher_assist(acceleration) for acceleration in accelerations]
    )
    checks = {
        "peak_speed": np.max(speeds) <= 3.0 + 1e-9,
        "peak_acceleration": np.max(np.abs(accelerations)) <= 1.5 + 1e-9,
        "final_stop": abs(speeds[-1]) <= 1e-9,
        "distance": profile.final_distance < 12.5,
        "pusher_assist_bounded": 0.04 < np.max(pusher_estimates) < 0.05,
    }
    print(f"profile_seconds={profile.profile_seconds:.3f}")
    print(f"final_distance={profile.final_distance:.3f}m")
    print(f"peak_speed={np.max(speeds):.3f}m/s")
    print(f"peak_acceleration={np.max(np.abs(accelerations)):.3f}m/s^2")
    print(f"expected_peak_px4_pusher_assist={np.max(pusher_estimates):.4f}")
    for name, passed in checks.items():
        print(f"{name}={'PASS' if passed else 'FAIL'}")
    if not all(checks.values()):
        raise SystemExit("offline_gate=FAIL")
    print("offline_gate=PASS")


if __name__ == "__main__":
    main()
