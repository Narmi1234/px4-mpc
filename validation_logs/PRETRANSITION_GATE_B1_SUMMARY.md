# Standard VTOL Gate B1 — prihvaćeni rezultat

Status: **PASS**  
Datum: `2026-08-26`  
ULog: `PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-26/05_52_44.ulg`  
ULog SHA-256: `28243c05fdde5a82821ee309fe088de9c5fc455fbc763b59b702446eb017926c`

Raw ULog nije kopiran u repo jer zauzima približno `57 MiB`. Ovaj zapis i hash
identificiraju prihvaćeni izvorni log.

## Konfiguracija

```text
vehicle:                 Gazebo standard_vtol
PX4 branch:              nmpc-external-pusher
test mode:               pretransition_5mps, MC cijelo vrijeme
target speed:            5.0 m/s
reference acceleration:  0.40 m/s^2
hold:                    3.0 s
NMPC/PX4 pusher limit:   0.15
collective:              validated hover feedback; NMPC collective shadow-only
Offboard timeout:        49.0 s PX4 vremena
```

## ULog rezultat

```text
offboard_duration=49.044s
peak_forward_speed=5.076m/s
peak_horizontal_speed=5.076m/s
final_horizontal_speed=0.034m/s
peak_calibrated_airspeed=5.359m/s
peak_commanded_pusher=0.1500
peak_pusher_actuator=0.1500
final_pusher_actuator=0.0000
commanded_collective_range=[0.5041,0.5241]
lift_motor_range=[0.5032,0.5244]
PX4 pusher parameters=[enabled=1,max=0.15,slew=0.10]
max_altitude_error=0.264m
max_vertical_speed=0.081m/s
max_tilt=4.55deg
max_cross_track=0.177m
solver_failures=0
VTOL state=MC cijelo vrijeme
ulog_gate=PASS
```

## Collective/lift rezultat za B2

ULog-binned collective komanda:

```text
hover / 0.0–0.5 m/s:  0.5198–0.5201
4.0–4.5 m/s:          0.5092
4.5–5.0 m/s:          0.5096
5 m/s hold:            0.5100
final settle:          0.5196
```

Izmjereno efektivno MC rasterećenje pri 5 m/s je približno `0.010`, dok raw
zero-pitch plant predviđa približno `0.022`. Model zato ne smije direktno
komandovati trim-corridor collective. B2 počinje konzervativnim kontinuiranim
rasporedom: nula ispod 3 m/s, validiranih `0.010` na 5 m/s i najviše `0.020`
na 8 m/s, uz postojeći altitude feedback i hard MC collective minimum.

## Zaključak

Gate B1 je zatvoren i ne ponavlja se radi tuninga. Dokazana granica je NMPC
pusher-speed kontrola do 5 m/s, stvarni motor 5, validan airspeed i siguran
povratak u hover, bez transition komande. Naredni korak je implementacija i
offline provjera Gate B2 na 8 m/s; još nema odobrene B2 live komande.
