# Gate A: 3 m/s NMPC pusher-feedback u MC režimu

Ovo je prvi zatvoreni pusher-speed gate. Letjelica cijelo vrijeme mora ostati
multicopter; ovaj test ne šalje VTOL transition komandu.

## Šta test dokazuje

NMPC dobija glatku pozicijsku i forward-speed referencu, a iz 10-state modela
računa pusher i body-rate komande. Pusher više nije unaprijed zadani vremenski
puls. Provjereni vertical-hover regulator još zamjenjuje NMPC collective izlaz
i izolovano čuva visinu. Lift-unloading počinje tek u Gateu B.

Profil u PX4/Gazebo vremenu:

```text
0.5 s neutralni Offboard handover
2.0 s mirna level-attitude referenca
6.28 s glatko ubrzanje do 3.0 m/s
2.0 s hold
6.28 s glatko kočenje
najmanje 3.0 s završni hover
20.5 s ukupni Offboard timeout
```

Profil i timeout koriste rekonstruisano sirovo PX4 boot vrijeme. Svaki
`TimesyncStatus` direktno sidri sat preko `remote_timestamp + observed_offset`,
dok se između direktnih uzoraka sat interpolira lokalnim monotonic satom i
izmjerenim Gazebo real-time faktorom. DDS-prevedeni apsolutni timestampovi se
ne koriste za runtime scheduling ni freshness jer mogu skočiti pri promjeni
timesync offseta. Ulazak u Offboard postavlja tačnu nulu intervala, pa se ROS
trajanje može direktno porediti sa stvarnim ULog intervalom.

Gate A zadržava provjereni pitch autoritet potreban za ubrzanje i kočenje, ali
ograničava roll rate na `0.10 rad/s`. Live ULog je pokazao da širi roll envelope
pretvara malu cross-track grešku u bočnu oscilaciju i prekoračenje ukupne
horizontalne brzine.

Gate A koristi zasebno generisan OCP sa pusher granicom `0.10`. Nije dovoljno
samo odsjeći izlaz na `0.10`: generički transition OCP dopušta `0.60`, pa bi
NMPC predviđao šest puta veći autoritet od komande koju PX4 stvarno izvršava.

Nominalni put je `24.85 m`. Potrebno je najmanje `40 m` slobodnog prostora
ispred nosa.

## PASS kriteriji

```text
2.5 <= peak forward speed <= 3.5 m/s
peak total horizontal speed <= 3.5 m/s
final horizontal speed <= 0.35 m/s
0.05 <= actual pusher peak <= 0.105
final actual pusher <= 0.005
max altitude error <= 0.30 m
max vertical speed <= 0.75 m/s
max tilt <= 10 deg
max cross-track <= 1.0 m
VTOL state = MC cijelo vrijeme
solver failures = 0
abort_reason = pusher_forward_test_timeout
```

Skripta i node ne prihvataju izmijenjen profil kao Gate A.

## 0. Offline provjera

PX4, Gazebo, Agent i QGC ne trebaju raditi:

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/setup_standard_vtol_nmpc.bash
PYTHONPATH=px4_mpc:${PYTHONPATH} .venv/bin/python \
  tools/simulate_pusher_forward_gate.py
```

Mora završiti sa `offline_gate=PASS`. Referentni rezultat implementacije je:

```text
solver_failures=0
max_speed_m_s=3.380
max_horizontal_speed_m_s=3.380
final_speed_m_s=0.002
max_altitude_error_m=0.237
max_vertical_speed_m_s=0.126
max_tilt_deg=6.57
max_pusher=0.100
max_cross_track_m=0.000
final_pusher=0.0002
solve_time_p99_ms≈4
offline_gate=PASS
```

## 1. Jednokratni ROS build

```bash
cd /home/imran/Repositories/px4-mpc
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --packages-select px4_mpc --symlink-install
```

Build mora završiti bez greške. Nakon builda zatvoriti taj terminal ili
nastaviti tačno sa source komandama navedenim ispod.

## 2. Terminal 1: custom PX4/Gazebo

Ne aktivirati `.venv`:

```bash
cd /home/imran/Repositories/PX4-Autopilot
git branch --show-current
make px4_sitl gz_standard_vtol
```

Branch mora biti:

```text
nmpc-external-pusher
```

U PX4 shellu, prije armiranja:

```text
param set VT_EXT_PUSH_EN 1
param set VT_EXT_PUSH_MAX 0.10
param set VT_EXT_PUSH_SLEW 0.10
param show VT_EXT_PUSH_EN
param show VT_EXT_PUSH_MAX
param show VT_EXT_PUSH_SLEW
```

Ne nastavljati ako parametri ne postoje ili prikazane vrijednosti nisu
`1`, `0.10`, `0.10`.

## 3. Terminal 2: DDS Agent

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/setup_standard_vtol_nmpc.bash
microxrce_agent_install/bin/MicroXRCEAgent udp4 -p 8888
```

Terminal mora ostati otvoren.

## 4. Terminal 3: Gate A NMPC node

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash

ros2 launch px4_mpc standard_vtol_nmpc_launch.py \
  allow_offboard_output:=true \
  allow_external_pusher_output:=true \
  allow_pusher_forward_output:=true \
  pusher_forward_test_max_seconds:=20.5 \
  pusher_forward_target_speed:=3.0 \
  pusher_forward_acceleration:=0.75 \
  pusher_forward_hold_seconds:=2.0 \
  pusher_forward_start_delay_seconds:=2.0
```

Sačekati poruku i ostaviti terminal otvoren:

```text
Standard VTOL NMPC started in armed-capable guarded MC plus guarded external
pusher plus guarded 3 m/s pusher feedback mode
```

## 5. Terminal 4: komunikacija prije armiranja

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash

ros2 topic hz /fmu/out/vehicle_odometry
```

Kada se pojavi frekvencija, prekinuti samo provjeru sa `Ctrl+C`. Zatim:

```bash
ros2 service call /standard_vtol_nmpc/status std_srvs/srv/Trigger '{}'
```

Status prije leta mora imati:

```text
output_requested=False
offboard=False
solver_failures=0
state_age manji od 0.20s
state_px4_age manji od 0.20s
vertical_rate age manji od 0.20s i apsolutna vrijednost manja od 0.10 m/s
px4_clock=[sync=True,...]
pusher_forward_profile=[speed=3.0,accel=0.75,hold=2.0]
nmpc_pusher_max=0.100
```

`airspeed=[...]` se mora pojaviti u statusu. Gate A ga bilježi, ali zbog male
brzine još ne blokira let ako PX4 prijavi synthetic/ground-minus-wind izvor.
Vertikalni feedback koristi `VehicleLocalPosition.z_deriv`, odnosno derivaciju
iste pozicije koju altitude watchdog prati. ULog je pokazao da je pouzdanija za
ovaj SITL od zasebnog EKF `vz` uzorka koji je jednom imao suprotan znak.

U aktivnom letu freshness watchdog koristi dvije nezavisne granice: PX4
plant-age `0.20 s` i wall transport-age `0.30 s`. Time kratka pauza cijelog
sporijeg Gazebo/DDS toka ne proizvodi lažni abort, dok nestanak samo odometryja
uz PX4 sat koji napreduje ostaje strogi fail. Završni status bilježi oba
maksimalna gapa.

## 6. QGroundControl

1. Vozilo mora biti u MC konfiguraciji i Position modeu.
2. Armirati i podići se na približno `10–15 m`.
3. Usmjeriti nos prema najmanje `40 m` čistog prostora.
4. Potpuno pustiti komande i čekati najmanje 5 s.
5. Provjeriti da nema penjanja, spuštanja ni yaw kretanja.
6. Držati QGC spreman za ručni izbor Position moda.

## 7. Terminal 4: jedan live Gate A

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_pusher_forward_gate.bash
```

Skripta će prije testa ponovo tražiti potvrdu prikazanih PX4 vrijednosti
`1`, `0.10`, `0.10`. To nije samo formalna provjera: ROS može uredno slati
`thrust_body[0]`, ali će PX4 ostaviti pusher actuator neaktivan kada je
`VT_EXT_PUSH_EN=0`.

Ne pozivati enable servis ručno i ne pokretati drugi gate u istom letu. Skripta
prati PX4 vrijeme do završetka, pa sporiji Gazebo može zahtijevati više od 20.5
wall sekundi.

Node uzima sirovo PX4 boot vrijeme samo iz direktnog para
`TimesyncStatus.remote_timestamp + observed_offset` i interpolira ga između
uzoraka. Prevedeni DDS epoch timestampovi nisu dozvoljeni za scheduling ni
plant-age jer se timesync offset tokom SITL-a mijenja i može lažno proglasiti
svježu odometriju zastarjelom ili prerano završiti gate.

Očekivani kraj:

```text
abort_reason=pusher_forward_test_timeout
test_mode=pusher_forward
profile_phase=settle_hover
last_offboard_duration=20.5s
solver_failures=0
ROS_GATE=PASS
```

ULog `offboard_duration` poslije zatvaranja loga također mora biti najmanje
`20.0 s`; ROS rezultat sam po sebi nije dovoljan.

Svaki drugi `abort_reason` je FAIL. Odmah sačuvati cijeli završni status i ne
ponavljati test prije analize.

## 8. ULog potvrda

Sletjeti, disarmirati i ugasiti PX4/Gazebo sa `Ctrl+C` da se ULog zatvori.
Pronaći najnoviji log:

```bash
find /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log \
  -type f -name '*.ulg' -printf '%T@ %s %p\n' | sort -nr | head
```

Analizirati tačnu prikazanu putanju:

```bash
cd /home/imran/Repositories/px4-mpc
/usr/bin/python3 tools/analyze_pusher_forward_ulog.py \
  /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/YYYY-MM-DD/HH_MM_SS.ulg
```

Prihvaćamo Gate A tek kada završi sa:

```text
duration=PASS
forward_speed=PASS
final_speed=PASS
pusher_peak=PASS
pusher_returned_zero=PASS
altitude=PASS
vertical_speed=PASS
tilt=PASS
cross_track=PASS
mc_only=PASS
ulog_gate=PASS
```

## 9. Poslije testa

Ne kopirati veliki ULog automatski u repo. Prvo ćemo sačuvati mali tekstualni
sažetak i odlučiti treba li raw log. U sljedećem PX4 pokretanju vratiti potvrđeni
limit dok ne počne Gate B:

```text
param set VT_EXT_PUSH_MAX 0.05
param set VT_EXT_PUSH_EN 0
```

Tek nakon ROS i ULog PASS rezultata prelazimo na Gate B (`5 m/s`, zatim
`8 m/s`) i uvodimo lift-unloading.
