# Standard VTOL trim corridor

## Kako znamo ravnotežna stanja

Trim nije kopiran iz ULoga niti ručno pogođen. Za svaku zadanu horizontalnu
brzinu `V` puni, prethodno validirani `StandardVtolGazeboModel` evaluira stanje

```text
v_world = [V, 0, 0]
omega_body = [0, 0, 0]
```

i traži varijable

```text
pitch, collective_lift, pusher, elevator_trim
```

tako da vrijedi

```text
ax_world = 0
az_world = 0
pitch_angular_acceleration = 0
```

Posljednja jednačina se koristi od 5 m/s, kada aerodinamička površina ima
dovoljno autoriteta. Ispod 5 m/s elevator nije efikasan; mali balansirajući
moment daje PX4 multirotor control allocator diferencijalnim lift-motorima.

Problem ima više mogućih ravnoteža. Dinamičko programiranje bira cijeli niz,
ne svaku tačku izolovano. Cost preferira:

- rasterećenje lift motora kako raste airspeed;
- ograničen pusher, pitch i elevator;
- male promjene između susjednih brzina;
- monoton pad collective-lift komande.

Posebno se računa wing-borne grana sa `collective_lift=0`. Ona postaje izvediva
na 10 m/s. To odgovara arhitekturi u kojoj krilo nosi težinu, a PX4 koristi
elevator/elevone da prati NMPC body-rate komandu.

## Generisanje

Nisu potrebni PX4, QGC, ROS 2 niti aktivan `.venv`:

```bash
cd /home/imran/Repositories/px4-mpc
MPLCONFIGDIR=/tmp/matplotlib-px4-mpc python3 \
  tools/generate_standard_vtol_trim_corridor.py \
  --output results/standard_vtol_trim_corridor
```

Rezultati su:

```text
results/standard_vtol_trim_corridor/
├── trim_corridor.csv
├── trim_corridor.yaml
└── trim_corridor.png
```

CSV sadrži i reziduale punog planta. Generator prekida ako ne postoji
ravnotežna tačka u granicama:

```text
-20 deg <= pitch <= 15 deg
0 <= collective_lift <= 0.65
0 <= pusher <= 0.60
-45 deg <= elevator <= 45 deg
```

## Trenutni ključni trimovi

| Airspeed | Pitch | Lift | Pusher | Elevator |
|---:|---:|---:|---:|---:|
| 0 m/s | 0.00° | 0.5201 | 0.0000 | 0.00° |
| 5 m/s | -8.75° | 0.4351 | 0.2902 | 43.32° |
| 10 m/s | -7.32° | 0.0000 | 0.2657 | 41.63° |
| 12 m/s | -4.05° | 0.0000 | 0.2660 | 25.14° |
| 15 m/s | -1.36° | 0.0000 | 0.2675 | 12.91° |
| 18 m/s | 0.10° | 0.0000 | 0.2696 | 6.74° |
| 22 m/s | 1.20° | 0.0000 | 0.2728 | 2.33° |

Pitch koristi Gazebo FLU konvenciju u postojećem modelu; negativna vrijednost
ovdje predstavlja nose-up trim. U ROS/PX4 nodeu mora se koristiti postojeća
ENU/FLU ↔ NED/FRD konverzija, ne direktno kopirati znak.

## Šta corridor jeste, a šta nije

Corridor je:

- referenca i warm start budućeg NMPC-a;
- provjera da postoje fizički izvedive ravnoteže kroz raspon brzina;
- nominalna raspodjela lift/pusher opterećenja.

Corridor nije:

- open-loop sekvenca koju treba poslati motorima;
- dokaz da se između tačaka može trenutno preći;
- zamjena za constraints, slew-rate limite i NMPC feedback;
- direktna elevator komanda. Elevator kolona potvrđuje da PX4 rate loop ima
  dovoljan aerodinamički autoritet.

Najveći trenutni translacijski rezidual kroz svih 23 tačke je ispod
`1e-8 m/s²`. CasADi 10-state model, acados OCP i zaštićeni hover-Offboard su
naknadno implementirani. Prošli su 30-sekundni hover gate i ograničena MC
horizontalna referenca od `2 m/s`. Sljedeći gate uvodi pusher i početak lift
schedulea pri maloj brzini, još uvijek prije PX4 VTOL transition komande.
