# Standard VTOL: prvi pusher-assist gate (3 m/s)

## Šta ovaj korak radi

Da, sada povećavamo forward brzinu i prvi put palimo pusher, ali konzervativno:

- PX4 ostaje u multicopter (`MC`) konfiguraciji;
- nema `VEHICLE_CMD_DO_VTOL_TRANSITION` komande;
- glatka referenca ide do `3 m/s`, koči i zaustavlja se;
- maksimalno ubrzanje je `1.5 m/s²`;
- nominalni put je `12.425 m`;
- NMPC cijelo vrijeme računa shadow rješenje;
- PX4 position controller prati putanju i njegov ugrađeni VTOL pusher-assist
  zakon upravlja pusherom.

Ova podjela je nužna. PX4 `Standard::fill_actuator_outputs()` u čistom MC modu
namjerno odbacuje body-X thrust iz `VehicleRatesSetpoint` poruke i zamjenjuje ga
internim `_pusher_throttle`. Zato bi direktno povećavanje NMPC pusher ulaza
uticalo na offline model, ali ne i na stvarnu Gazebo letjelicu.

Za airframe `4004_gz_standard_vtol`, PX4 već postavlja
`VT_FWD_THRUST_EN=4`. Kod maksimalnog ubrzanja ovog profila, PX4-ova formula sa
`VT_PITCH_MIN=-5 deg` i `VT_FWD_THRUST_SC=0.7` predviđa mali peak pusher od oko
`0.0448`. Stvarna vrijednost se prihvata tek nakon provjere ULoga.

Ovaj test **nije dokaz da NMPC direktno upravlja pusherom** i nije VTOL
tranzicija. On validira PX4 pusher integraciju, višu brzinu i aerodinamički
početak transition corridora prije promjene kontrolnog interfejsa.

## Sigurnosne granice

- ulazak samo iz armed, mirnog MC hovera;
- cilj `3.0 m/s`, hard limit `3.8 m/s`;
- pusher ostaje pod PX4 VTOL kontrolerom;
- bez promjene VTOL stanja iz MC;
- visinska greška najviše `0.5 m`;
- vertikalna brzina najviše `0.75 m/s`;
- nagib najviše `25 deg`;
- cross-track i tracking error najviše `1.5 m`;
- geofence od `-1 m` iza starta do `2 m` iza nominalnog kraja;
- automatski povratak u Position mode nakon `13 s`.

Potrebno je najmanje **20 m čistog prostora ispred nosa**. Test radi na oko
`12 m` visine.

## 0. Offline provjera

PX4, ROS, Gazebo i QGC nisu potrebni:

```bash
cd /home/imran/Repositories/px4-mpc
PYTHONPATH=px4_mpc /usr/bin/python3 tools/check_pusher_assist_gate.py
```

Mora završiti sa:

```text
expected_peak_px4_pusher_assist=0.0448
offline_gate=PASS
```

## 1. Terminal 1: PX4/Gazebo

Ne aktivirati projektnu `.venv` u ovom terminalu:

```bash
cd /home/imran/Repositories/PX4-Autopilot
make px4_sitl gz_standard_vtol
```

U PX4 shellu, prije armiranja, samo provjeriti parametre:

```text
param show VT_FWD_THRUST_EN
param show VT_FWD_THRUST_SC
param show VT_PITCH_MIN
```

Očekivano je redom `4`, `0.7` i `-5`. Ako vrijednosti nisu takve, ne pokretati
gate i ne mijenjati ih napamet.

## 2. Terminal 2: DDS Agent

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/setup_standard_vtol_nmpc.bash
microxrce_agent_install/bin/MicroXRCEAgent udp4 -p 8888
```

## 3. Terminal 3: NMPC i safety supervisor

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash

ros2 launch px4_mpc standard_vtol_nmpc_launch.py \
  allow_offboard_output:=true \
  allow_pusher_assist_output:=true \
  pusher_assist_test_max_seconds:=13.0 \
  pusher_assist_target_speed:=3.0 \
  pusher_assist_acceleration:=1.5 \
  pusher_assist_hold_seconds:=1.0 \
  pusher_assist_start_delay_seconds:=2.0
```

Sačekati poruku i ostaviti terminal otvoren:

```text
Standard VTOL NMPC started in armed-capable guarded MC plus guarded PX4 pusher-assist mode
```

## 4. QGroundControl priprema

1. Ostaviti letjelicu u MC konfiguraciji i Position modeu.
2. Armirati i podići se na približno `12 m`.
3. Usmjeriti nos prema najmanje 20 m čistog prostora.
4. Potpuno pustiti komande i čekati najmanje 5 s.
5. Ne pokretati gate dok se visina, brzina ili yaw još mijenjaju.
6. Držati QGC spreman za ručni izbor Position moda.

## 5. Terminal 4: jedan automatski gate

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_pusher_assist_gate.bash
```

Ne pozivati servis ručno i ne ponavljati gate u istom letu. Uspješan završni
status mora sadržavati:

```text
abort_reason=pusher_assist_test_timeout
test_mode=pusher_assist
profile_phase=settle_hover
configured_timeout=13.0s
output_interface=px4_position_with_vtol_pusher_assist
solver_failures=0
```

Svaki drugi `abort_reason` znači da test nije prošao.

## 6. Poslije testa i obavezna ULog potvrda

1. Sletjeti kroz QGC i disarmirati.
2. Ugasiti PX4/Gazebo sa `Ctrl+C` da se ULog zatvori i prestane rasti.
3. Pronaći najnoviji ULog:

```bash
find /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log \
  -type f -name '*.ulg' -printf '%T@ %p\n' | sort -nr | head
```

4. U narednu komandu kopirati stvarnu putanju najnovijeg fajla:

```bash
cd /home/imran/Repositories/px4-mpc
/usr/bin/python3 tools/analyze_pusher_assist_ulog.py \
  /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/YYYY-MM-DD/HH_MM_SS.ulg
```

Prihvatamo test tek kada završi sa:

```text
pusher_engaged=PASS
speed_reached=PASS
altitude_span=PASS
mc_only=PASS
ulog_gate=PASS
```

Tek nakon toga projektujemo kontrolni interfejs za parcijalnu PX4 tranziciju.
Ne podižemo brzinu na 5–10 m/s i ne šaljemo transition komandu prije pregleda
ovog ULoga.
