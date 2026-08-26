# Gate C: stock PX4 transition uz NMPC shadow

Gate A, B1 i B2 su prihvaćeni. Gate C je prvi stvarni front/back transition,
ali **nije NMPC-controlled transition**. PX4 sam upravlja pusherom, lift-motor
blendom, elevonima i VTOL state machineom. NMPC samo računa predloženu komandu
i ne objavljuje nijedan Offboard setpoint.

## Šta dokazujemo

```text
MC -> TRANSITION_TO_FW -> FW -> TRANSITION_TO_MC -> MC
VT_EXT_PUSH_EN=0, odnosno stock PX4 je vlasnik pushera
airspeed na ulasku u FW >= 10 m/s
stock transition cycle bez failsafea
stvarni lift-motor blend i pusher iz ULoga
NMPC radi u shadowu bez solver greške
siguran povratak u MC
```

Gate C dopušta stock PX4 envelope do `12 m` gubitka visine i `60 deg` tilta.
To nisu ciljevi za NMPC. Gate D će koristiti strože granice `2 m` i `20 deg`.

## Jednokratni ROS build

```bash
cd /home/imran/Repositories/px4-mpc
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --packages-select px4_mpc --symlink-install
```

## Terminal 1 — PX4/Gazebo

Ne aktivirati `px4-mpc/.venv`:

```bash
cd /home/imran/Repositories/PX4-Autopilot
git branch --show-current
make px4_sitl gz_standard_vtol
```

Branch ostaje `nmpc-external-pusher`, ali se external pusher obavezno gasi. U
PX4 shellu prije armiranja:

```text
param set VT_EXT_PUSH_EN 0
param show VT_EXT_PUSH_EN
param show VT_ARSP_BLEND
param show VT_ARSP_TRANS
param show VT_TRANS_MIN_TM
```

`VT_EXT_PUSH_EN` mora prikazati `0`. Ne mijenjati stock transition parametre.

## Terminal 2 — DDS Agent

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/setup_standard_vtol_nmpc.bash
microxrce_agent_install/bin/MicroXRCEAgent udp4 -p 8888
```

## Terminal 3 — NMPC shadow-only

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
ros2 launch px4_mpc standard_vtol_nmpc_launch.py
```

Startup poruka mora završiti sa `shadow-only mode`. Ne dodavati nijedan
`allow_*_output:=true` argument.

## Terminal 4 — provjera prije leta

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash

ros2 topic hz /fmu/out/vehicle_odometry
```

Kada je frekvencija stabilna, prekinuti samo `hz` sa `Ctrl+C`, pa:

```bash
ros2 service call /standard_vtol_nmpc/status std_srvs/srv/Trigger '{}'
```

Mora pisati:

```text
output_requested=False
offboard=False
solver_failures=0
total_solver_failures=0
state_age < 0.20s
airspeed=[...,available=True,...]
```

Negativan CAS u hoveru je dozvoljen ako je `available=True`.

## QGC i transition

1. Vozilo mora biti MC i Position mode.
2. Armirati i podići na najmanje `30 m`.
3. Usmjeriti nos prema najmanje `500 m` čistog prostora.
4. Čekati najmanje `10 s` u stabilnom hoveru.
5. U QGC izabrati `Transition to Fixed Wing`. Ako opcija nije vidljiva, u
   Terminalu 1, na `pxh>` promptu, unijeti `commander transition`.
6. Ne slati Offboard servis niti external-pusher komandu.
7. Kada PX4 završi front transition, ostati FW `10–15 s`.
8. Zatražiti `Transition to Multicopter` u QGC, odnosno ponovo unijeti
   `commander transition` na `pxh>` promptu.
9. Čekati stabilan MC hover najmanje `10 s`, zatim sletjeti i disarmirati.

Ako PX4 prijavi failsafe/quad-chute ili se približi tlu, ne forsirati novi
transition: vratiti MC ako je moguće i sletjeti.

## Terminal 4 — shadow rezultat

Nakon povratka u MC, prije gašenja Terminala 3:

```bash
ros2 service call /standard_vtol_nmpc/status std_srvs/srv/Trigger '{}'
```

Za PASS moraju ostati:

```text
output_requested=False
offboard=False
solver_failures=0
total_solver_failures=0
```

## ULog provjera

Nakon slijetanja, disarma i gašenja PX4/Gazebo:

```bash
find /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log \
  -type f -name '*.ulg' -printf '%T@ %s %p\n' | sort -nr | head
```

Zamijeniti putanju stvarnim najnovijim logom:

```bash
cd /home/imran/Repositories/px4-mpc
/usr/bin/python3 tools/analyze_transition_shadow_ulog.py \
  /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/YYYY-MM-DD/HH_MM_SS.ulg
```

Mora završiti sa:

```text
complete_state_sequence=PASS
fw_entry_airspeed=PASS
no_failsafe=PASS
stock_px4_owns_pusher=PASS
lift_blend_observed=PASS
transition_shadow_ulog_gate=PASS
```

Zatim generisati model-residual report bez kopiranja ULoga u repo:

```bash
/usr/bin/python3 tools/validate_standard_vtol_ulog.py \
  /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/YYYY-MM-DD/HH_MM_SS.ulg \
  --output /tmp/standard_vtol_gate_c_model \
  --no-plots

sed -n '1,120p' /tmp/standard_vtol_gate_c_model/report.md
```

Gate C se prihvata tek kada ULog analyzer prođe, NMPC ima nula ukupnih solver
grešaka i report sadrži sve četiri VTOL faze. Tada se hash i sažetak zapisuju u
repo, a počinje implementacija Gate D state machinea.
