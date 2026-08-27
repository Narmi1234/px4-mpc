# Gate D attempt 03 — diagnosed FAIL

```text
ULog: /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-27/16_23_20.ulg
size: 59,464,883 bytes
sha256: 326da1092f2060f24b1a2156793276ef683eea1a928a64720e779eabf334cbf3
result: FAIL (vertical_speed_limit)
```

Recovery again returned the aircraft to MC before Position. Active-flight
maxima were tilt `5.85 deg`, vertical speed `1.689 m/s`, altitude error
`1.789 m`, CAS `10.589 m/s`, pusher `0.300`, minimum collective `0.000`, and
zero solver failures. The final status pitch after recovery is not an active
Gate D maximum.

## Diagnosis

The Attempt 02 pitch correction worked. The remaining failure was slow passage
through the lift blend: CAS needed about `3.25 s` to move from `8.3` to
`10.1 m/s` while collective faded. At CAS `10.06 m/s`, collective was `0.100`,
vertical speed was already `1.30 m/s`, and pitch was still a bounded
approximately `-2.9 deg`. This is primarily a thrust/time-in-blend problem,
not a new pitch or solver failure.

The successful stock Gate C crossed the same airspeed range in about `0.5 s`,
with front-transition pusher approaching `0.85`. The next NMPC profile keeps
the already validated MC limit `0.30`, then allows a bounded `0.60` front
feedforward with `0.33 command/s` slew. The corrected offline run peaks at only
`0.443` pusher and passes with altitude error `0.838 m`, tilt `6.28 deg`, speed
`12.034 m/s`, zero solver failures and final MC.
