# Standard VTOL NMPC: 2 m/s multicopter forward gate

Status: **passed in SITL on 2026-08-16**. The accepted measurements and the
boundary of what this test proves are recorded in
[`STANDARD_VTOL_MC_FORWARD_RESULTS.md`](STANDARD_VTOL_MC_FORWARD_RESULTS.md).

Ovaj test **ne radi VTOL tranziciju**. PX4 mora cijelo vrijeme ostati u
multicopter konfiguraciji, a pusher komanda je programski zaključana na nulu.
Cilj je odvojeno potvrditi pokretnu horizontalnu referencu, heading/frame
mapiranje, ubrzanje, kočenje i završno zadržavanje pozicije.

## Profil koji će letjelica pratiti

Referenca se postavlja u smjeru nosa letjelice u trenutku poziva servisa:

1. jedna sekunda PX4 Offboard prestreama;
2. 0.5 s neutralnog hover handovera;
3. 2 s mirnog NMPC warm-upa;
4. glatka kosinusna rampa do 2 m/s za približno 3.14 s;
5. 1 s na 2 m/s;
6. glatko kočenje približno 3.14 s;
7. završni hover do automatskog timeouta na 15 s.

Nominalni put je `8.28 m`. Potrebno je barem 12 m slobodnog prostora ispred
nosa. Profil ne vraća letjelicu na početnu tačku; zaustavlja je na novoj hover
tački.

## Sigurnosne granice

- ulazak samo iz mirnog, armed MC hovera;
- lift `0.48–0.56`, lift slew `0.10/s`;
- pusher uvijek `0.0`;
- body-rate granice `0.20/0.20/0.15 rad/s`;
- horizontalna brzina najviše `2.7 m/s`;
- horizontalni tracking error najviše `1.5 m`;
- cross-track najviše `1.5 m`;
- putanja od `-1 m` iza starta do `2 m` iza nominalnog kraja;
- visinska greška `0.5 m`, vertikalna brzina `0.75 m/s`, nagib `25 deg`;
- stale odometry, PX4 failsafe, izlazak iz MC moda ili tri solver greške odmah
  traže Position mode.

Timeout se smatra uspjehom samo ako je dostignuto najmanje `1.6 m/s`, završna
brzina je ispod `0.35 m/s`, a završna pozicijska greška ispod `0.75 m`.

## 0. Offline provjera prije leta

PX4, Gazebo i QGC nisu potrebni:

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/setup_standard_vtol_nmpc.bash
PYTHONPATH=px4_mpc:${PYTHONPATH} .venv/bin/python \
  tools/simulate_mc_forward_gate.py
```

Mora završiti sa:

```text
solver_failures=0
offline_gate=PASS
```

Referentni rezultat prije prvog live testa je `2.089 m/s` maksimalne i
`0.0026 m/s` završne brzine, `0.168 m` maksimalne horizontalne tracking greške,
`0.066 m` visinske greške, `6.93 deg` nagiba i pusher `0.0`.

## 1. Jednokratni build

```bash
cd /home/imran/Repositories/px4-mpc
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --packages-select px4_mpc --symlink-install
```

## 2. Terminal 1: PX4/Gazebo

Ne aktivirati projektnu `.venv` u ovom terminalu:

```bash
cd /home/imran/Repositories/PX4-Autopilot
make px4_sitl gz_standard_vtol
```

## 3. Terminal 2: DDS Agent

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/setup_standard_vtol_nmpc.bash
microxrce_agent_install/bin/MicroXRCEAgent udp4 -p 8888
```

## 4. Terminal 3: NMPC

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
ros2 launch px4_mpc standard_vtol_nmpc_launch.py \
  allow_offboard_output:=true \
  hover_offboard_max_seconds:=30.0 \
  mc_forward_test_max_seconds:=15.0 \
  mc_forward_target_speed:=2.0 \
  mc_forward_acceleration:=1.0 \
  mc_forward_hold_seconds:=1.0 \
  mc_forward_start_delay_seconds:=2.0
```

Prvi start nakon promjene modela može ponovo generisati acados solver. Sačekati
poruku:

```text
Standard VTOL NMPC started in armed-capable guarded MC mode
```

Ovaj terminal mora ostati otvoren. Ako se vrati shell prompt ili se pojavi
`process has died`, NMPC više nije pokrenut i gate skripta nema servis kojem se
može javiti. Ponovo pokrenuti komandu iz ovog koraka i sačuvati eventualni
Python traceback iz terminala.

## 5. Terminal 4: DDS provjera

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
ros2 topic hz /fmu/out/vehicle_odometry
```

Mora se pojaviti frekvencija. Prekinuti samo ovu provjeru sa `Ctrl+C`.

## 6. QGroundControl priprema

1. Ostaviti letjelicu u multicopter konfiguraciji i Position modeu.
2. Armirati i podići se na približno `10 m`.
3. Usmjeriti nos prema najmanje 12 m slobodnog prostora.
4. Potpuno pustiti komande i čekati najmanje 5 s.
5. Ne pokretati gate dok se letjelica penje, spušta ili yaw još mijenja.

## 7. Terminal 4: jedan automatski live gate

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_mc_forward_gate.bash
```

Paziti da je putanja tačno `px4-mpc`, a ne `px4-mpcpc`. Ako servis nije
dostupan, skripta sada nakon 8 s prekida bez slanja Offboard komande. U tom
slučaju ponovo pokrenuti Terminal 3, sačekati njegovu startup poruku i tek onda
ponoviti ovaj korak u Terminalu 4.

Ne pozivati servis ručno i ne pokretati gate drugi put u istom letu. Držati QGC
spreman; kod opasnog nagiba ili kretanja odmah ručno izabrati **Position mode**.
Ako readiness ili launch konfiguracija nisu ispravni, skripta se odmah prekida
bez čekanja i bez traženja Offboard moda.

Uspješan završni status mora sadržavati:

```text
test_mode=mc_forward
profile_phase=settle_hover
configured_timeout=15.0s
last_offboard_duration=15.0s
abort_reason=mc_forward_test_timeout
```

Svaki drugi `abort_reason` znači da gate nije prošao. `maxima` u istom statusu
pokazuje dostignutu brzinu, cross-track, tracking, visinsku grešku i nagib.

## 8. Poslije testa

1. Sletjeti kroz QGC i disarmirati.
2. Ugasiti PX4/Gazebo sa `Ctrl+C` da ULog ne nastavi rasti.
3. Ne izvoditi novi test dok se node log i ULog ne analiziraju.

Tek nakon uspješnog ovog gatea dodaje se koordinisana PX4 VTOL transition
komanda. Pusher se u ovom testu nikada ne koristi.
