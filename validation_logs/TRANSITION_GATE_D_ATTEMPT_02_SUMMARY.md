# Gate D attempt 02 — diagnosed FAIL

```text
ULog: /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-27/06_18_25.ulg
size: 46,844,295 bytes
sha256: 31bbcdc431ac2edc0d0d12f5667a3ac70273c38148c29bf2fad73e92aae9775c
result: FAIL (altitude_error)
```

Recovery worked: PX4 accepted the front and back requests and returned to MC
before Position mode. The state sequence was `MC -> TRANSITION_TO_FW -> MC`;
the aircraft did not enter confirmed FW.

ROS maxima before recovery:

```text
offboard duration:    31.05 s
forward speed:        11.527 m/s
calibrated airspeed:  11.639 m/s
altitude error:       2.032 m
tilt:                 15.51 deg
vertical speed:       3.341 m/s
commanded pusher:     0.300
minimum collective:   0.032
solver failures:      0
```

## ULog diagnosis

The explicit lift blend worked, but exposed a phase error in the reference.
During front transition, pitch first moved to approximately `+6 deg`, then
reversed through `-15 deg` as the NMPC pitch-rate command saturated and the
lift collective faded. The aircraft then descended rapidly. This is not a
slow-airspeed failure: CAS exceeded `10 m/s` and peaked above `11.6 m/s`.

The identified pitch/elevator table is a wing-borne trim corridor, not the
front-transition transient. A successful stock PX4 transition in the Gate C
ULog stayed approximately level (`-0.8..0.1 deg`) and reached FW in `2.58 s`.

## Correction before another live attempt

- Front transition now uses a level pitch reference; FW trim begins only
  after PX4 confirms FW.
- A bounded pitch-rate governor retains NMPC altitude authority inside a
  four-degree corridor but limits rate to `0.10 rad/s` and slew to
  `0.20 rad/s^2`.
- Front-transition recovery begins at `10 deg` tilt or `1.5 m/s` vertical
  speed instead of waiting for the broader Gate D limits.
- Corrected offline Gate D passes in `57.20 s`: altitude error `1.187 m`, tilt
  `8.83 deg`, maximum speed `11.753 m/s`, zero solver failures and final MC.
