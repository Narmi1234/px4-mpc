# Standard VTOL 2 m/s MC-forward gate: passed

## Accepted SITL run

Date: 2026-08-16

The first guarded moving-reference test passed. PX4 remained in multicopter
configuration, the pusher command remained locked at zero, and NMPC controlled
the aircraft in Offboard for the configured 15-second interval before
automatically requesting Position mode.

Final service result:

```text
abort_reason=mc_forward_test_timeout
test_mode=mc_forward
profile_phase=settle_hover
last_offboard_duration=15.03s
solver_failures=0
maxima=[forward_speed=2.018,cross_track=0.214,
        horizontal_tracking=0.225,altitude=0.187,tilt_deg=7.37]
tracking_error_pv=[-0.0235,-0.1106,0.0646,-0.0006,0.0162,-0.0115]
displacement_xy=[8.2308,-0.8017]
```

The PX4 mode-change acknowledgement was `command=176,result=0`. The node log
also records the deliberate terminal condition and Position-mode request:

```text
NMPC stopped: reason=mc_forward_test_timeout, offboard_elapsed=15.03s,
mode=mc_forward, phase=settle_hover,
maxima=[forward_speed=2.018, cross_track=0.214,
horizontal_tracking=0.225, altitude=0.187, tilt_deg=7.37];
Position mode requested
```

Candidate ULog for the accepted run (kept outside Git because it is large):

```text
/home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/
2026-08-16/12_28_20.ulg
```

At inspection time it was approximately 26 MB. The earlier
`12_09_59.ulg` is a 217 MB pre-gate session and is not acceptance evidence.

## Gate decision

Passed criteria:

- Offboard remained active for the full configured duration;
- peak forward speed exceeded the required `1.6 m/s`;
- the aircraft stopped at the final reference;
- horizontal tracking, cross-track, altitude and tilt stayed inside every
  configured safety limit;
- the solver had no failures;
- fallback to Position mode was commanded and acknowledged.

This validates moving horizontal references, ENU/FLU to NED/FRD command signs,
acceleration, braking and bumpless return to Position mode at low speed. It
does not yet validate pusher thrust, lift-motor unloading, the trim corridor
above 2 m/s, or a PX4 VTOL transition command.

## Next implementation gate

The next software increment is a separately guarded low-speed transition-entry
test. It must introduce the pusher and trim-corridor lift schedule gradually
while PX4 remains in MC mode, then brake and return to hover. Only after that
gate passes may the node issue a PX4 VTOL transition command. Do not reuse the
2 m/s service with manually changed launch parameters; the node intentionally
rejects such a configuration.
