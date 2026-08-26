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

## B1 — prvi 5 m/s MC let

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

## Između B1 i B2

Iz B1 ULoga se računa koliko je collective regulator stvarno morao smanjiti
lift pri istoj visini i brzini. Tek taj izmjereni MC podatak postaje ograničeni
feedforward lift-unloading raspored. Raspored mora:

- početi od nule ispod `3 m/s`;
- biti kontinuiran i monotono rasterećivati lift motore;
- imati feedback korekciju visine iznad feedforwarda;
- vratiti puni hover collective pri kočenju i prije završnog hovera;
- biti ograničen tako da jedan model mismatch ne može ugasiti lift motore.

## B2 — 8 m/s MC pre-transition let

B2 se implementira tek nakon B1 ULog PASS-a. Početni pusher limit je `0.20`,
ali se može smanjiti ako B1 pokaže da nije potreban. Na `8 m/s` se prvi put
aktivira validirani lift-unloading feedforward uz altitude feedback.

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

Tek B1 i B2 PASS otvaraju Gate C, gdje stock PX4 izvodi transition, a NMPC radi
shadow prediction bez preuzimanja transition aktuatora.
