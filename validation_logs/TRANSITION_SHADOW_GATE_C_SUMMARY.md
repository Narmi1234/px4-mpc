# Standard VTOL Gate C — stock PX4 transition shadow rezultat

Status: **PASS za PX4/ULog/model checkpoint**
Datum: `2026-08-26`
ULog: `PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-08-26/17_14_25.ulg`
ULog SHA-256: `267e12b1b92d53b17d3dd32043fe3553877830a317129131c623ffdbbae344b7`

Raw ULog nije kopiran u repo jer zauzima približno `208 MiB`.

## Gate rezultat

```text
transition_state_history=[3,1,4,2,3]
transition_cycle_duration=38.908s
fw_entry_airspeed=11.213m/s
peak_calibrated_airspeed=19.115m/s
peak_ground_speed=20.236m/s
altitude_loss=6.988m
maximum_tilt=47.00deg
pusher_peak=0.8494
lift_motor_range=[0.0047,0.5368]
VT_EXT_PUSH_EN=0
failsafe=False
transition_shadow_ulog_gate=PASS
```

PX4 je u FW Position/loiter režimu letio u krug. Zato `47 deg` maksimalnog
tilta i dio bočne/uglovne dinamike pripadaju normalnom FW zaokretu, a ne NMPC
oscilaciji. Gate C nije koristio Offboard niti external-pusher izlaz.

## Model reziduali, samo interval 720–770 s

```text
overall linear acceleration RMSE N/E/D = 0.171 / 0.201 / 0.293 m/s^2
front-transition RMSE N/E/D             = 0.005 / 0.189 / 0.101 m/s^2
back-transition RMSE N/E/D              = 0.200 / 0.210 / 0.159 m/s^2
fixed-wing RMSE N/E/D                   = 0.199 / 0.233 / 0.364 m/s^2
```

Veliki angular-acceleration reziduali tokom FW kruženja nisu direktni state
model Gate D kontrolera: 10-state NMPC šalje body-rate reference, a PX4 ostaje
vlasnik unutrašnje rate petlje, elevona i alokacije.

## Zaključak za Gate D

Model ima ispravan translacijski trend kroz obje tranzicije i može se koristiti
za prvi ograničeni Gate D uz PX4-authoritative VTOL state/blend. Stock PX4 je
tokom ciklusa izgubio `6.988 m` visine; Gate D zato ne smije kopirati stock
profil i mora imati state-dependent recovery, altitude-loss limit i stabilan
airspeed uslov prije front-transition zahtjeva.

Shadow node log sadrži uredan startup i nema crash/solver exception. Završni
`total_solver_failures` service odgovor nije arhiviran, pa se taj brojač ne
navodi kao zaseban dokaz. Gate D skripta mora automatski sačuvati njegov finalni
status; Gate C stock let se zbog toga ne ponavlja.
