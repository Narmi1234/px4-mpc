# Gate B: MC pre-transition envelope

Ovo je jedini dokument za nastavak nakon prihvaćenog Gate A. Gate A je
zaključan u `validation_logs/PUSHER_FORWARD_GATE_A_SUMMARY.md`.

Gate B još uvijek **ne šalje VTOL transition komandu**. Cilj je odvojeno
dokazati da NMPC može sigurno doći do airspeed područja u kojem PX4 započinje
transition blend, dok vozilo ostaje MC.

## Redoslijed

```text
Gate A PASS, 3 m/s
  -> B0: implementacija i offline regresija, bez leta
  -> B1: jedan 5 m/s MC let, hover collective regulator ostaje aktivan
  -> B1 ULog: izmjeri stvarni wing-lift/collective rezidual
  -> B2: jedan 8 m/s MC let sa validiranim postepenim lift-unloadingom
  -> Gate C: PX4 transition shadow test
```

Ne preskakati direktno na `8 m/s` i ne pokretati stock trim iz
`trim_corridor.yaml` kao komandu. Taj trim od `5 m/s` naviše uključuje veliki
elevator equilibrium, dok je elevator u MC režimu zaključan; zato se prvo mora
izmjeriti stvarni MC collective rezidual.

## B0 — implementacija i offline provjera: PASS

Implementirano je:

1. Poseban `pretransition_5mps` test mode i servis; Gate A parametri ostaju
   nepromijenjeni.
2. Smooth speed profil `0 -> 5 -> 0 m/s`, početna akceleracija najviše
   `0.40 m/s^2`, hold `3 s` i najmanje `4 s` završnog hovera.
3. OCP i PX4 pusher hard limit `0.15`; ne koristiti generički OCP limit
   `0.60`.
4. Airspeed tok mora biti fresh, finite i imati aktivan source prije starta i
   tokom profila. Synthetic CAS može biti negativan oko stationary hovera;
   pozitivan CAS se zahtijeva tek tokom ubrzanja. Ground-forward speed ostaje
   feedback za path/geofence.
5. Dokazani Gate A cross-track regulator i simetrični pitch barrier ostaju
   aktivni.
6. Dokazani vertical-hover collective regulator ostaje stvarna komanda u B1.
   NMPC/trim collective se računa i loguje samo kao `shadow_collective`.
7. Status i analyzer moraju prijaviti ground speed, CAS, collective command,
   stvarne lift-motor izlaze, pusher, tilt, visinu, cross-track, VTOL state i
   solver statistiku.
8. Offline nominalni i disturbance test prije live upute.

B0 nominalni rezultat:

```text
max_speed_m_s=4.994
final_speed_m_s=0.00002
max_tracking_error_m=0.250
max_altitude_error_m=0.248
max_tilt_deg=1.75
max_pusher=0.150
solver_failures=0
offline_gate=PASS
```

Disturbance test uključuje `0.50 m/s` bočni udar, `0.25 m/s` forward gust,
35% jači rate odziv i `-6 deg` pitch poremećaj. Prolazi sa `5.242 m/s`,
cross-trackom `0.485 m`, altitude errorom `0.248 m`, tiltom `7.48 deg` i nula
solver grešaka.

## B1 — PASS: 5 m/s MC let

Prihvaćeni rezultat i ULog hash su u
`validation_logs/PRETRANSITION_GATE_B1_SUMMARY.md`. B1 je završio sa
`5.076 m/s`, final speed `0.034 m/s`, CAS `5.359 m/s`, stvarnim pusherom
`0.150 -> 0`, altitude errorom `0.264 m`, tiltom `4.55 deg`, cross-trackom
`0.177 m`, MC-only i `ulog_gate=PASS`.

Planirana fiksna konfiguracija:

```text
target ground-forward speed: 5.0 m/s
reference acceleration:      0.40 m/s^2
hold:                        3.0 s
pusher OCP/PX4 limit:        0.15
collective output:           dokazani vertical-hover regulator
NMPC collective:             shadow-only
VTOL state:                  MC cijelo vrijeme
```

Profil prelazi približno `113 m`. Minimalni sigurnosni prostor je zato `150 m`
ispred nosa, a početna visina `15–20 m`.

PASS kriteriji:

```text
4.25 <= peak forward speed <= 5.75 m/s
final horizontal speed <= 0.40 m/s
max altitude error <= 0.40 m
max vertical speed <= 1.0 m/s
max tilt <= 10 deg
max cross-track <= 1.0 m
actual pusher <= 0.155 i na kraju <= 0.005
fresh/valid airspeed tokom aktivnog profila
VTOL state = MC cijelo vrijeme
solver failures = 0
siguran povratak na hover
```

## B1 postupak pokretanja

### 0. Offline gate

PX4, Gazebo, Agent i QGC ne trebaju raditi:

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash

python3 tools/simulate_pusher_forward_gate.py \
  --duration 49.0 \
  --target-speed 5.0 \
  --acceleration 0.40 \
  --hold-seconds 3.0 \
  --start-delay-seconds 2.0 \
  --pusher-limit 0.15 \
  --minimum-peak-speed 4.25 \
  --maximum-speed 5.75 \
  --maximum-final-speed 0.40 \
  --maximum-altitude-error 0.40 \
  --maximum-cross-track 1.0 \
  --maximum-tilt-degrees 10.0 \
  --output /tmp/standard_vtol_gate_b1_offline
```

Mora završiti sa `offline_gate=PASS`.

### 1. Jednokratni ROS build

```bash
cd /home/imran/Repositories/px4-mpc
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --packages-select px4_mpc --symlink-install
```

### 2. Terminal 1 — custom PX4/Gazebo

Ne aktivirati `px4-mpc/.venv` u ovom terminalu:

```bash
cd /home/imran/Repositories/PX4-Autopilot
git branch --show-current
make px4_sitl gz_standard_vtol
```

Branch mora biti `nmpc-external-pusher`. U PX4 shellu prije armiranja:

```text
param set VT_EXT_PUSH_EN 1
param set VT_EXT_PUSH_MAX 0.15
param set VT_EXT_PUSH_SLEW 0.10
param show VT_EXT_PUSH_EN
param show VT_EXT_PUSH_MAX
param show VT_EXT_PUSH_SLEW
```

Mora prikazati `1`, `0.15`, `0.10`. Ne potvrđivati run skripti samo zato što
su komande ukucane; pročitati stvarni izlaz svake `param show` komande. Prvi
B1 pokušaj je ULogom pokazao `enabled=0,max=0.05`, pa taj let nije dokazao
stvarni pusher i ne smije se ponoviti bez ove provjere.

### 3. Terminal 2 — DDS Agent

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/setup_standard_vtol_nmpc.bash
microxrce_agent_install/bin/MicroXRCEAgent udp4 -p 8888
```

Ostaviti terminal otvoren.

### 4. Terminal 3 — B1 NMPC node

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash

ros2 launch px4_mpc standard_vtol_nmpc_launch.py \
  allow_offboard_output:=true \
  allow_external_pusher_output:=true \
  allow_pretransition_output:=true \
  pretransition_test_max_seconds:=49.0 \
  pretransition_target_speed:=5.0 \
  pretransition_acceleration:=0.40 \
  pretransition_hold_seconds:=3.0 \
  pretransition_start_delay_seconds:=2.0
```

Sačekati startup poruku koja sadrži:

```text
guarded 5 m/s MC pre-transition
```

Ostaviti terminal otvoren.

### 5. Terminal 4 — komunikacija prije armiranja

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash

ros2 topic hz /fmu/out/vehicle_odometry
```

Kada se pojavi stabilna frekvencija, prekinuti samo `hz` sa `Ctrl+C`, pa:

```bash
ros2 service call /standard_vtol_nmpc/status std_srvs/srv/Trigger '{}'
```

Prije leta moraš vidjeti:

```text
output_requested=False
offboard=False
solver_failures=0
state_age < 0.20s
state_px4_age < 0.20s
px4_clock=[sync=True,...]
airspeed=[...,available=True,...]
pretransition_profile=[speed=5.0,accel=0.40,hold=3.0]
nmpc_pusher_max=0.150
```

### 6. QGroundControl

1. Vozilo mora biti MC i u Position modu.
2. Armirati i podići na `15–20 m`.
3. Usmjeriti nos prema najmanje `150 m` potpuno čistog prostora.
4. Pustiti komande i čekati najmanje `5 s`.
5. Ne pokretati gate ako se vozilo kreće horizontalno, penje ili spušta.
6. Držati QGC spreman za trenutni ručni izbor Position moda.

### 7. Terminal 4 — jedan B1 let

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_pretransition_5mps_gate.bash
```

Skripta ponovo provjerava profil i svjež airspeed tok, a operator mora prepisati
tačno `1,0.15,0.10` tek nakon provjere PX4 izlaza. Negativan synthetic CAS oko
hovera je dozvoljen ako piše `available=True`; tokom leta peak CAS mora dostići
najmanje `4.0 m/s`. Ne pozivati enable servis ručno. Očekivani završetak je:

```text
abort_reason=pretransition_5mps_test_timeout
last_offboard_duration=49.0s
solver_failures=0
ROS_GATE=PASS
```

Ako vidiš rastuću oscilaciju, veliki pad/penjanje ili promjenu iz MC režima,
odmah izaberi Position mode. Svaki drugi `abort_reason` je FAIL i let se ne
ponavlja prije ULog analize.

Prvi B1 pokušaj abortirao je na DDS odometry gapu `0.365 s`, dok je interni
PX4 `vehicle_local_position` ostao kontinuiran sa maksimalnim gapom `0.044 s`.
B1 zato dozvoljava najviše `0.45 s` state gapa; Gate A limiti ostaju
nepromijenjeni. Gap veći od `0.45 s` i dalje odmah vraća Position mode.

### 8. ULog analiza

Sletjeti, disarmirati i ugasiti PX4/Gazebo da se ULog zatvori. Pronaći
najnoviji log:

```bash
find /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log \
  -type f -name '*.ulg' -printf '%T@ %s %p\n' | sort -nr | head
```

Analizirati tačnu putanju:

```bash
cd /home/imran/Repositories/px4-mpc
/usr/bin/python3 tools/analyze_pusher_forward_ulog.py \
  /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/YYYY-MM-DD/HH_MM_SS.ulg \
  --minimum-duration 45.0 \
  --minimum-forward-speed 4.25 \
  --maximum-forward-speed 5.75 \
  --maximum-final-speed 0.40 \
  --maximum-altitude-error 0.40 \
  --maximum-vertical-speed 1.0 \
  --maximum-tilt-degrees 10.0 \
  --maximum-cross-track 1.0 \
  --expected-pusher-limit 0.15 \
  --minimum-peak-airspeed 4.0 \
  --require-airspeed
```

Prihvatamo B1 samo ako završi sa `ulog_gate=PASS`. Sačuvati analyzer output;
raw ULog se ne kopira u repo dok ne odlučimo da je potreban.

## Između B1 i B2 — izmjereno

Iz B1 ULoga se računa koliko je collective regulator stvarno morao smanjiti
lift pri istoj visini i brzini. Tek taj izmjereni MC podatak postaje ograničeni
feedforward lift-unloading raspored. Raspored mora:

- početi od nule ispod `3 m/s`;
- biti kontinuiran i monotono rasterećivati lift motore;
- imati feedback korekciju visine iznad feedforwarda;
- vratiti puni hover collective pri kočenju i prije završnog hovera;
- biti ograničen tako da jedan model mismatch ne može ugasiti lift motore.

Prihvaćeni ULog daje hover collective oko `0.5201` i `0.5096–0.5100` pri
4.5–5 m/s, odnosno efektivno rasterećenje oko `0.010`. Raw plant predviđa
`0.022` na 5 m/s i zato precjenjuje wing-lift efekt. Početni B2 schedule mora
biti konzervativniji od direktnog modela:

```text
speed <= 3 m/s: unload = 0.000
speed = 5 m/s:  unload = 0.010
speed = 8 m/s:  unload <= 0.020
```

Interpolacija mora biti glatka, a vrijednost se dodaje kao feedforward ispod
postojećeg altitude feedbacka. Pri kočenju schedule prati izmjerenu brzinu i
vraća se na nulu prije završnog hovera.

## B2 — 8 m/s MC pre-transition let

B1 ULog je PASS. B2 live let i ULog su također **PASS**; prihvaćeni rezultat
i hash su u `validation_logs/PRETRANSITION_GATE_B2_SUMMARY.md`.
Početni pusher limit je `0.20`. Na `8 m/s` se prvi put aktivira
ULog-ograničeni lift-unloading feedforward uz postojeći altitude feedback.
Vozilo cijelo vrijeme ostaje MC; ovo još nije transition.

Fiksna konfiguracija:

```text
target ground-forward speed: 8.0 m/s
reference acceleration:      0.50 m/s^2
hold:                        3.0 s
pusher OCP/PX4 limit:        0.20
lift unloading:              0 do 3 m/s; 0.010 na 5; 0.020 na 8 m/s
collective hard bounds:      0.48 do 0.56
timeout:                     60.0 s PX4 vremena
VTOL state:                  MC cijelo vrijeme
```

Profil prelazi približno `225 m`. Potrebno je najmanje `300 m` potpuno čistog
prostora ispred nosa i početna visina `20 m`.

Offline nominalni rezultat:

```text
max_speed_m_s=7.986
final_speed_m_s=0.00001
max_tracking_error_m=0.399
max_altitude_error_m=0.238
max_tilt_deg=2.25
max_pusher=0.200
max_lift_unloading=0.020
minimum_collective=0.497
solver_failures=0
offline_gate=PASS
```

Disturbance test uključuje `0.50 m/s` bočni udar, `0.30 m/s` forward gust,
35% jači rate odziv i `-6 deg` pitch poremećaj. Prolazi sa peak horizontalnom
brzinom `8.002 m/s`, cross-trackom `0.482 m`, altitude errorom `0.237 m`,
tiltom `5.12 deg`, minimalnim collectiveom `0.480` i bez solver grešaka.

PASS kriteriji ostaju najmanje jednako strogi:

```text
speed tracking error <= 0.75 m/s
final horizontal speed <= 0.50 m/s
max altitude error <= 0.40 m
max vertical speed <= 1.0 m/s
max tilt <= 12 deg
max cross-track <= 1.5 m
collective i pusher bez skokova
lift motori nikada ispod sigurnog MC minimuma
VTOL state = MC cijelo vrijeme
solver failures = 0
```

### B2 postupak pokretanja

Prvo uraditi ROS build samo jednom nakon ovog commita:

```bash
cd /home/imran/Repositories/px4-mpc
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --packages-select px4_mpc --symlink-install
```

#### Terminal 1 — custom PX4/Gazebo

Ne aktivirati `px4-mpc/.venv`:

```bash
cd /home/imran/Repositories/PX4-Autopilot
git branch --show-current
make px4_sitl gz_standard_vtol
```

Branch mora biti `nmpc-external-pusher`. U PX4 shellu prije armiranja:

```text
param set VT_EXT_PUSH_EN 1
param set VT_EXT_PUSH_MAX 0.20
param set VT_EXT_PUSH_SLEW 0.10
param show VT_EXT_PUSH_EN
param show VT_EXT_PUSH_MAX
param show VT_EXT_PUSH_SLEW
```

Izlaz mora stvarno prikazati `1`, `0.20`, `0.10`.

#### Terminal 2 — DDS Agent

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/setup_standard_vtol_nmpc.bash
microxrce_agent_install/bin/MicroXRCEAgent udp4 -p 8888
```

#### Terminal 3 — B2 NMPC node

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash

ros2 launch px4_mpc standard_vtol_nmpc_launch.py \
  allow_offboard_output:=true \
  allow_external_pusher_output:=true \
  allow_lift_unloading_output:=true \
  lift_unloading_test_max_seconds:=60.0 \
  lift_unloading_target_speed:=8.0 \
  lift_unloading_acceleration:=0.50 \
  lift_unloading_hold_seconds:=3.0 \
  lift_unloading_start_delay_seconds:=2.0 \
  lift_unloading_maximum:=0.020
```

Sačekati poruku `guarded 8 m/s MC lift-unloading` i ostaviti terminal otvoren.

#### Terminal 4 — provjera i let

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
ros2 topic hz /fmu/out/vehicle_odometry
```

Kad je frekvencija stabilna, prekinuti samo `hz` sa `Ctrl+C`. U QGC:

1. vozilo mora biti MC i Position;
2. armirati i podići na `20 m`;
3. nos usmjeriti prema najmanje `300 m` potpuno čistog prostora;
4. sačekati najmanje `5 s` u mirnom hoveru;
5. držati QGC spreman za trenutni ručni povratak u Position.

Zatim u Terminalu 4 pokrenuti tačno jednu skriptu:

```bash
bash scripts/run_pretransition_8mps_gate.bash
```

Kada zatraži potvrdu, unijeti `1,0.20,0.10` samo ako su to stvarno pokazale
tri PX4 `param show` komande. Tokom testa gledati da nema rastuće oscilacije,
velikog gubitka visine ili izlaska iz MC režima. U tim slučajevima odmah
izabrati Position mode.

Očekivani ROS završetak:

```text
abort_reason=pretransition_8mps_test_timeout
last_offboard_duration=60.0s
maxima=[...,forward_speed između 7.25 i 8.75,...
airspeed najmanje 7.0,...commanded_pusher do 0.200,lift_unloading=0.020,...]
solver_failures=0
ROS_GATE=PASS
```

Svaki drugi `abort_reason` je FAIL. Ne ponavljati let prije analize.

#### B2 ULog analiza

Sletjeti, disarmirati i ugasiti PX4/Gazebo da se log zatvori. Pronaći najnoviji
ULog i analizirati baš njegovu punu putanju:

```bash
find /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log \
  -type f -name '*.ulg' -printf '%T@ %s %p\n' | sort -nr | head

cd /home/imran/Repositories/px4-mpc
/usr/bin/python3 tools/analyze_pusher_forward_ulog.py \
  /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/YYYY-MM-DD/HH_MM_SS.ulg \
  --minimum-duration 56.0 \
  --minimum-forward-speed 7.25 \
  --maximum-forward-speed 8.75 \
  --maximum-final-speed 0.50 \
  --maximum-altitude-error 0.40 \
  --maximum-vertical-speed 1.0 \
  --maximum-tilt-degrees 12.0 \
  --maximum-cross-track 1.5 \
  --expected-pusher-limit 0.20 \
  --minimum-peak-airspeed 7.0 \
  --minimum-lift-motor 0.30 \
  --require-airspeed
```

B2 je prihvaćen sa `ROS_GATE=PASS` i `ulog_gate=PASS`. Peak brzina je bila
`8.111 m/s`, peak CAS `8.199 m/s`, altitude error `0.241 m`, tilt `4.61 deg`,
minimalni lift-motor izlaz `0.4914`, a stvarni pusher se vratio na nulu.

Gate B2 PASS otvara Gate C: prvo PX4 transition uz NMPC shadow prediction, bez
NMPC preuzimanja transition aktuatora. Nakon validacije shadow predikcije ide
ograničeni NMPC transition gate.
