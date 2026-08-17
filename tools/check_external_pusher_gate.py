#!/usr/bin/env python3
"""Offline check for the first NMPC-to-PX4 external-pusher pulse."""

import numpy as np

from px4_mpc.models.external_pusher_profile import ExternalPusherProfile


def main() -> None:
    profile = ExternalPusherProfile()
    times = np.linspace(0.0, 12.0, 1201)
    commands = np.array([profile.sample(time).command for time in times])
    slew = np.max(np.abs(np.diff(commands) / np.diff(times)))
    checks = {
        "peak_command": np.max(commands) <= 0.05 + 1e-12,
        "slew": slew <= 0.02 + 1e-12,
        "starts_zero": commands[0] == 0.0,
        "ends_zero": commands[-1] == 0.0,
        "settle_margin": 12.0 - profile.profile_seconds >= 3.0,
    }
    print(f"profile_seconds={profile.profile_seconds:.2f}")
    print(f"peak_command={np.max(commands):.3f}")
    print(f"peak_slew={slew:.3f}/s")
    for name, passed in checks.items():
        print(f"{name}={'PASS' if passed else 'FAIL'}")
    if not all(checks.values()):
        raise SystemExit("offline_gate=FAIL")
    print("offline_gate=PASS")


if __name__ == "__main__":
    main()
