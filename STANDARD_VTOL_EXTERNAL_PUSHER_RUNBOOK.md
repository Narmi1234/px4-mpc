# Standard VTOL NMPC: external-pusher interface gate

## Šta je promijenjeno

PX4 branch `nmpc-external-pusher` u repozitoriju
`/home/imran/Repositories/PX4-Autopilot` dodaje opt-in interfejs kojim NMPC u
armed Offboard body-rate modu upravlja pusherom kroz
`VehicleRatesSetpoint.thrust_body[0]`.

NMPC izlaz ostaje:

```text
[collective_lift, pusher, roll_rate, pitch_rate, yaw_rate]
```

PX4 i dalje zatvara body-rate petlje, raspoređuje collective lift na četiri
motora, upravlja actuator allocatorom i zadržava failsafe logiku. NMPC ne šalje
pojedinačne brzine motora.

Novi PX4 parametri su:

```text
VT_EXT_PUSH_EN    default 0     # interfejs ugašen
VT_EXT_PUSH_MAX   default 0.05  # hard limit
VT_EXT_PUSH_SLEW  default 0.10  # dodatni PX4 slew limit [1/s]
```

Interfejs je aktivan samo ako su istovremeno ispunjeni svi uslovi:

- `VT_EXT_PUSH_EN=1`;
- vozilo je armed;
- nav mode je Offboard;
- Offboard koristi body-rate nivo;
- Standard VTOL nije u punom FW modu.

Stale setpoint stariji od 200 ms, NaN komanda, disarm ili izlazak iz traženog
Offboard interfejsa odmah uklanjaju external pusher override. Negativna komanda
se clampuje na nulu, a pozitivna na `VT_EXT_PUSH_MAX`.

## Prvi gate

Ovaj test još ne povećava zadanu forward brzinu. On izolovano dokazuje cijeli
put komande:

```text
NMPC node -> ROS 2 -> uXRCE-DDS -> PX4 VTOL controller -> allocator -> motor 5
```

Komanda je samo:

```text
2.0 s nula
2.5 s rampa 0.00 -> 0.05
2.0 s hold na 0.05
2.5 s rampa 0.05 -> 0.00
3.0 s završni hover
```

PX4 hard limit je `0.05`, a ROS slew samo `0.02/s`. Vozilo cijelo vrijeme mora
ostati u MC konfiguraciji. NMPC stabilizira hover svojim collective-lift i
body-rate komandama dok se pusher puls izvršava.

## 0. Offline provjera

```bash
cd /home/imran/Repositories/px4-mpc
PYTHONPATH=px4_mpc /usr/bin/python3 tools/check_external_pusher_gate.py
```

Mora završiti sa `offline_gate=PASS`.

## 1. Terminal 1: custom PX4/Gazebo

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
param set VT_EXT_PUSH_MAX 0.05
param set VT_EXT_PUSH_SLEW 0.10
param show VT_EXT_PUSH_EN
param show VT_EXT_PUSH_MAX
param show VT_EXT_PUSH_SLEW
```

Ako PX4 kaže da parametar ne postoji, pokrenut je pogrešan branch ili stari
binary. Ne nastavljati test.

## 2. Terminal 2: DDS Agent

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/setup_standard_vtol_nmpc.bash
microxrce_agent_install/bin/MicroXRCEAgent udp4 -p 8888
```

## 3. Terminal 3: NMPC

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash

ros2 launch px4_mpc standard_vtol_nmpc_launch.py \
  allow_offboard_output:=true \
  allow_external_pusher_output:=true \
  external_pusher_test_max_seconds:=12.0 \
  external_pusher_peak:=0.05 \
  external_pusher_slew:=0.02 \
  external_pusher_hold_seconds:=2.0 \
  external_pusher_start_delay_seconds:=2.0
```

Sačekati i ostaviti terminal otvoren:

```text
Standard VTOL NMPC started in armed-capable guarded MC plus guarded external pusher mode
```

Ako se ovaj terminal zatvori, padne ili se prekine sa `Ctrl+C`, servis ne
postoji i Terminal 4 ne može pokrenuti test. Provjera prije armiranja:

```bash
ros2 service list | grep standard_vtol_nmpc
```

Mora prikazati najmanje `/standard_vtol_nmpc/status` i
`/standard_vtol_nmpc/enable_external_pusher_test`.

## 4. QGroundControl

1. Ostaviti vozilo u MC konfiguraciji i Position modeu.
2. Armirati i podići se na približno `10 m`.
3. Ostaviti najmanje 10 m prostora ispred nosa.
4. Pustiti komande i čekati najmanje 5 s da hover bude miran.
5. Držati QGC spreman za ručni izbor Position moda.

## 5. Terminal 4: jedan test

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_external_pusher_gate.bash
```

Skripta prvo provjerava da Terminal 3 zaista radi. Poruka `NMPC node is not
available` znači da treba ponovo pokrenuti i ostaviti otvoren Terminal 3; tada
nikakav Offboard ni pusher setpoint nije poslan.

Uspješan završni status mora sadržavati:

```text
abort_reason=external_pusher_test_timeout
test_mode=external_pusher
profile_phase=settle_hover
last_offboard_duration=12.0s
solver_failures=0
commanded_pusher=0.050
```

To potvrđuje da je ROS strana završila sigurno, ali motor 5 se prihvata tek iz
ULoga.

## 6. ULog potvrda

Sletjeti, disarmirati i ugasiti PX4/Gazebo da se ULog zatvori. Pronaći
najnoviji fajl:

```bash
find /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log \
  -type f -name '*.ulg' -printf '%T@ %p\n' | sort -nr | head
```

Zatim analizirati stvarnu putanju:

```bash
cd /home/imran/Repositories/px4-mpc
/usr/bin/python3 tools/analyze_external_pusher_ulog.py \
  /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/YYYY-MM-DD/HH_MM_SS.ulg
```

Prihvatamo interfejs tek kada dobijemo:

```text
pusher_peak=PASS
pusher_returned_zero=PASS
speed_limit=PASS
altitude_span=PASS
mc_only=PASS
ulog_gate=PASS
```

Napomena: NMPC trenutno mjeri ovaj gate u ROS wall vremenu, a PX4 ULog koristi
Gazebo simulation time. Ako Gazebo radi sporije od realnog vremena, 12.0 s koje
prijavi NMPC može u ULogu biti kraće; analyzer zato prihvata najmanje 10.0 s,
ali i dalje nezavisno zahtijeva dostignut pusher peak, povratak na nulu, sigurne
brzinu i visinu te MC stanje tokom cijelog intervala.

Nakon testa, u sljedećem PX4 pokretanju ponovo ugasiti eksperimentalni
interfejs dok ne bude potreban:

```text
param set VT_EXT_PUSH_EN 0
```

Ako ovaj gate prođe, sljedeći korak je zatvoreni NMPC forward-speed gate do
`3 m/s` u kojem NMPC, a ne PX4 pusher-assist, istovremeno komanduje collective
liftom, pusherom i body rateovima.
