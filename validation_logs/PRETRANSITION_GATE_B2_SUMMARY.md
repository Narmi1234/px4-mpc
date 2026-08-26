# Standard VTOL Gate B2 — prihvaćeni rezultat

Status: **PASS**
Datum: `2026-08-26`
ULog: `PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-26/06_28_37.ulg`
ULog SHA-256: `846e188308dab5f6ace8139a137ae6fc9b4e935b1a2d8277effb28a36a23a695`

Raw ULog nije kopiran u repo jer zauzima približno `94 MiB`. Ovaj zapis i hash
jednoznačno identificiraju prihvaćeni izvorni log.

## Konfiguracija

```text
vehicle:                 Gazebo standard_vtol
PX4 branch:              nmpc-external-pusher
test mode:               pretransition_8mps, MC cijelo vrijeme
target speed:            8.0 m/s
reference acceleration:  0.50 m/s^2
hold:                    3.0 s
NMPC/PX4 pusher limit:   0.20
lift unloading:          0 ispod 3 m/s; 0.010 na 5; max 0.020 na 8 m/s
collective bounds:       0.48 do 0.56
Offboard timeout:        60.0 s PX4 vremena
```

## ULog rezultat

```text
offboard_duration=60.144s
peak_forward_speed=8.111m/s
peak_horizontal_speed=8.111m/s
final_horizontal_speed=0.032m/s
peak_calibrated_airspeed=8.199m/s
peak_commanded_pusher=0.1792
peak_pusher_actuator=0.1792
final_pusher_actuator=0.0000
commanded_collective_range=[0.4919,0.5253]
lift_motor_range=[0.4914,0.5274]
minimum_lift_motor=0.4914
PX4 pusher parameters=[enabled=1,max=0.20,slew=0.10]
max_altitude_error=0.241m
max_vertical_speed=0.273m/s
max_tilt=4.61deg
max_cross_track=0.476m
solver_failures=0
VTOL state=MC cijelo vrijeme
ulog_gate=PASS
```

## Zaključak

Gate B2 je zatvoren i ne ponavlja se radi tuninga. Dokazana je stabilna NMPC
pusher-speed kontrola do `8.111 m/s`, konzervativni lift-unloading, sigurno
kočenje i povratak u hover. Naredni korak je Gate C: stock PX4 front/back
transition uz NMPC shadow računanje, bez Offboard izlaza i sa
`VT_EXT_PUSH_EN=0`.
