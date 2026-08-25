# Standard VTOL Gate A — prihvaćeni rezultat

Status: **PASS**  
Datum: `2026-08-25`  
ULog: `PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-25/18_49_22.ulg`  
ULog SHA-256: `33a2c502c549da25b98689cb0364937009f6051861c5ba64097ebb5ceb984a04`

Raw ULog nije kopiran u ovaj repo jer zauzima približno `25 MiB`. Ovaj sažetak
i hash trajno identificiraju prihvaćeni izvorni log dok postoji u PX4 log
direktoriju.

## Konfiguracija

```text
vehicle:                 Gazebo standard_vtol
PX4 branch:              nmpc-external-pusher
test mode:               pusher_forward, MC cijelo vrijeme
target speed:            3.0 m/s
reference acceleration:  0.50 m/s^2
hold:                    2.0 s
NMPC/PX4 pusher limit:   0.10
Offboard timeout:        26.5 s PX4 vremena
```

Aktivna stabilizacija uključuje speed-focused moving along-track referencu,
prigušenu cross-track/roll petlju, simetrični pitch control-barrier i dokazani
vertical-hover regulator.

## ULog rezultat

```text
offboard_duration=26.576s
peak_forward_speed=3.127m/s
peak_horizontal_speed=3.127m/s
final_horizontal_speed=0.053m/s
peak_calibrated_airspeed=3.682m/s
peak_commanded_pusher=0.0838
peak_pusher_actuator=0.0838
final_pusher_actuator=0.0000
max_altitude_error=0.200m
max_vertical_speed=0.096m/s
max_tilt=3.22deg
max_cross_track=0.663m
solver_failures=0
VTOL state=MC cijelo vrijeme
ulog_gate=PASS
```

## Zaključak

Gate A je zatvoren. Ne ponavljati ga radi daljeg tuninga i ne mijenjati njegove
limite. Dokazana granica je stabilan pusher-feedback do `3 m/s` u MC režimu,
bez lift-unloadinga i bez VTOL transition komande. Naredni rad počinje sa Gate
B1 na `5 m/s` prema `STANDARD_VTOL_GATE_B_RUNBOOK.md`.
