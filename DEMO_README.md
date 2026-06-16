# px4-mpc Local Demo Notes

Ovaj fajl je prakticni podsjetnik za lokalni setup, build i demo pokretanje
`px4-mpc` workspace-a na ovoj masini.

Workspace root:

```bash
/home/imran/Repositories/px4-mpc
```

Lokalno su dodani dependency/source folderi:

```text
px4_msgs/
px4-offboard/
acados/
.python_deps/
```

Nemoj ih commitati u upstream repo ako ne zelis vendored dependency-je u svom
git history-ju.

## 1. Setup

Ovaj dio je zajednicki za ROS/PX4 demo i za offline `standard_vtol_nmpc`
eksperimente.

### 1.1 Jednokratni system setup

Ovo se radi samo jednom na masini.

```bash
sudo apt update
sudo apt install software-properties-common curl git cmake build-essential python3-pip python3-venv -y
sudo add-apt-repository universe
sudo apt update
```

ROS 2 Jazzy za Ubuntu 24.04:

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install ros-jazzy-desktop ros-dev-tools -y
```

Ako `apt --fix-broken install` zapne na `libdart6.13` konfliktu sa OSRF/Gazebo
paketima, ukloni samo polu-instalirane Ubuntu DART dev pakete:

```bash
sudo apt remove libdart-dev libdart-external-odelcpsolver-dev libdart-external-convhull-3d-dev
sudo apt --fix-broken install
sudo dpkg --configure -a
```

### 1.2 Source dependency-ji

U workspace root-u:

```bash
cd /home/imran/Repositories/px4-mpc
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git
```

acados:

```bash
cd /home/imran/Repositories/px4-mpc
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
cmake -S . -B build -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=ON
cmake --build build --target install -j4
```

Python dependency-ji u lokalni folder:

```bash
cd /home/imran/Repositories/px4-mpc
python3 -m pip install --target .python_deps casadi numpy scipy matplotlib
python3 -m pip install --target .python_deps ./acados/interfaces/acados_template
```

acados `t_renderer`:

```bash
cd /home/imran/Repositories/px4-mpc
printf "y\n" | PYTHONPATH=$PWD/.python_deps ACADOS_SOURCE_DIR=$PWD/acados python3 -c "from acados_template.utils import get_tera; print(get_tera())"
```

### 1.3 Environment za svaki novi terminal

Pokreni ovo svaki put prije build/run komandi:

```bash
cd /home/imran/Repositories/px4-mpc

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export PYTHONPATH=$PWD/.python_deps:$PYTHONPATH
export ACADOS_SOURCE_DIR=$PWD/acados
export LD_LIBRARY_PATH=$PWD/acados/lib:$LD_LIBRARY_PATH
export MPLCONFIGDIR=$PWD/.cache/matplotlib
```

Ako `install/setup.bash` jos ne postoji, prvo uradi build.

### 1.4 Build workspace-a

```bash
cd /home/imran/Repositories/px4-mpc
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to px4_mpc
source install/setup.bash
```

Prvi build `px4_msgs` paketa moze trajati vise minuta jer generise veliki broj
ROS 2 poruka.

## 2. Quadrotor ROS/PX4 demo

Ovaj dio koristi ROS 2 paket `px4_mpc` i entry point `mpc_quadrotor`.

Napomena: originalni upstream README spominje `quadrotor_demo`, ali u ovom
repo-u stvarni entry point je:

```bash
ros2 run px4_mpc mpc_quadrotor
```

### 2.1 Pokretanje samo MPC node-a

Ovo starta MPC node bez RViz-a:

```bash
cd /home/imran/Repositories/px4-mpc

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export PYTHONPATH=$PWD/.python_deps:$PYTHONPATH
export ACADOS_SOURCE_DIR=$PWD/acados
export LD_LIBRARY_PATH=$PWD/acados/lib:$LD_LIBRARY_PATH
export MPLCONFIGDIR=$PWD/.cache/matplotlib

ros2 run px4_mpc mpc_quadrotor
```

### 2.2 Pokretanje RViz demo launch-a

```bash
cd /home/imran/Repositories/px4-mpc

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export PYTHONPATH=$PWD/.python_deps:$PYTHONPATH
export ACADOS_SOURCE_DIR=$PWD/acados
export LD_LIBRARY_PATH=$PWD/acados/lib:$LD_LIBRARY_PATH
export MPLCONFIGDIR=$PWD/.cache/matplotlib

ros2 launch px4_mpc mpc_quadrotor_launch.py
```

Launch pokrece:

- `px4_mpc/mpc_quadrotor`
- `px4_mpc/rviz_pos_marker`
- `px4_offboard/visualizer`
- `rviz2`

Ako RViz samo stoji, to je normalno dok PX4 SITL i DDS agent nisu pokrenuti.
MPC node ceka PX4 podatke na topicima:

```text
/fmu/out/vehicle_status
/fmu/out/vehicle_attitude
/fmu/out/vehicle_local_position
```

Provjera:

```bash
ros2 topic list | grep fmu
```

Ako nema `/fmu/out/...` topica, PX4 nije spojen na ROS 2.

### 2.3 Puni PX4 SITL demo

Za puni demo trebaju tri terminala.

Terminal 1: PX4 SITL. Za quadrotor koristi quadrotor Gazebo model, npr:

```bash
cd /home/imran/Repositories/PX4-Autopilot
make px4_sitl gz_x500
```

Terminal 2: DDS/micro agent.

Na ovoj masini je agent buildan lokalno iz source-a u:

```text
Micro-XRCE-DDS-Agent/
microxrce_agent_install/
```

Pokreni ga ovako:

```bash
cd /home/imran/Repositories/px4-mpc
export LD_LIBRARY_PATH=$PWD/microxrce_agent_install/lib:$LD_LIBRARY_PATH
$PWD/microxrce_agent_install/bin/MicroXRCEAgent udp4 -p 8888
```

Ako si agent instalirao globalno nekim drugim putem, onda moze raditi i kraca
komanda:

```bash
MicroXRCEAgent udp4 -p 8888
```

Za stariji README flow ponekad se vidi i:

```bash
micro-ros-agent udp4 --port 8888
```

Terminal 3: MPC + RViz.

```bash
cd /home/imran/Repositories/px4-mpc

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export PYTHONPATH=$PWD/.python_deps:$PYTHONPATH
export ACADOS_SOURCE_DIR=$PWD/acados
export LD_LIBRARY_PATH=$PWD/acados/lib:$LD_LIBRARY_PATH
export MPLCONFIGDIR=$PWD/.cache/matplotlib

ros2 launch px4_mpc mpc_quadrotor_launch.py
```

### 2.4 Sta treba vidjeti

Bez PX4 SITL-a:

- RViz se otvori.
- Node-ovi startaju.
- Nema stvarnog kretanja jer nema `/fmu/out/...` podataka.

Sa PX4 SITL-om i agentom:

- `ros2 topic list | grep fmu` prikazuje PX4 topic-e.
- MPC prima attitude/local position/status.
- RViz prikazuje referencu i predicted path.
- MPC objavljuje offboard control mode na `/fmu/in/offboard_control_mode`.
- Kad je PX4 u Offboard modu, MPC salje rate setpoint na
  `/fmu/in/vehicle_rates_setpoint`.

### 2.5 Ko pokrece dron i ko daje trajektoriju

`px4_mpc` ne arm-a dron i ne prebacuje PX4 u Offboard mode automatski.
Komandu za takeoff, arm i Offboard daje operator preko QGroundControl-a ili
PX4 shell-a.

Tipican redoslijed:

1. Pokreni agent.
2. Pokreni PX4 SITL.
3. Pokreni `px4_mpc` launch.
4. U QGroundControl klikni `Takeoff`.
5. Kad se dron stabilizuje, prebaci mode na `Offboard`.

Ako QGroundControl pise `Some modes hidden`, to obicno znaci da PX4 jos ne
prima offboard setpoint stream. Pricekaj par sekundi nakon pokretanja
`px4_mpc`, pa opet probaj. Provjera:

```bash
ros2 topic echo /fmu/in/offboard_control_mode
```

Ako se poruke ispisuju, PX4 bi trebao moci prihvatiti Offboard.

Alternativa iz PX4 SITL shell-a:

```bash
commander takeoff
commander mode offboard
```

Ovaj MPC node nema punu trajektoriju po defaultu. Ima jedan position setpoint,
inicijalno:

```text
x = 0.0, y = 0.0, z = 3.0
```

Setpoint se moze promijeniti preko ROS service-a:

```bash
ros2 service call /set_pose mpc_msgs/srv/SetPose "{pose: {position: {x: 2.0, y: 0.0, z: 3.0}, orientation: {w: 1.0}}}"
```

ili:

```bash
ros2 service call /set_pose mpc_msgs/srv/SetPose "{pose: {position: {x: 0.0, y: 2.0, z: 3.0}, orientation: {w: 1.0}}}"
```

Napomena: service trenutno vraca `result=False` i kad je setpoint prihvacen,
jer callback u kodu ne postavlja `response.result = True`. To je propust u
repo-u, ne znaci nuzno da service nije radio.

### 2.6 Sigurnosna napomena za quadrotor demo

Kad PX4 udje u `Offboard`, `px4_mpc` salje body-rate/thrust komande na:

```text
/fmu/in/vehicle_rates_setpoint
```

Ovaj controller nije "sigurni demo autopilot". Ako frame transformacije,
altitude znak, thrust mapping ili tuning ne odgovaraju SITL-u, dron moze
krenuti u neocekivanom smjeru. Drzi QGroundControl spreman za prebacivanje
nazad iz `Offboard` moda.

Ako dron krene nekontrolisano:

- U QGroundControl prebaci iz `Offboard` u `Hold` ili `Position`.
- Klikni `Land` ili `RTL`.
- Iz PX4 SITL shell-a koristi jednu od komandi:

```bash
commander mode hold
commander land
commander disarm
```

## 3. Standard VTOL

Ovdje postoje dvije razlicite stvari koje ne treba mijesati:

- `standard_vtol_nmpc`: offline Python/CasADi NMPC laboratorija za 6DOF standard
  VTOL transition.
- PX4 `gz_standard_vtol`: SITL model u PX4/Gazebo svijetu.
- `px4_mpc/mpc_standard_vtol`: ROS 2 offboard node koji sada spaja
  `standard_vtol_nmpc` NMPC na PX4 topic-e.

Trenutno stanje:

- Faza 1 radi offline: model, optimizacija i plotovi bez ROS-a/PX4-a.
- Faza 2 radi u ROS/PX4 offboard-u: NMPC cita PX4 stanje i salje
  `VehicleRatesSetpoint` na `/fmu/in/vehicle_rates_setpoint`.
- Dodatni node `standard_vtol_reference` daje referencu za standard VTOL.
- Dodatni node `standard_vtol_commander` salje offboard heartbeat, prebacuje
  PX4 u Offboard mode i arma letjelicu.

### 3.1 Standard VTOL Faza 1: offline NMPC

Ovo pokrece 6DOF standard VTOL model bez ROS-a, PX4-a, Gazebo-a i acados-a.

Stanje modela:

```text
[px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
```

Kontrole:

```text
[lift_thrust, pusher_thrust, roll_moment, pitch_moment, yaw_moment]
```

Pokretanje simulacije:

```bash
cd /home/imran/Repositories/px4-mpc
PYTHONPATH=$PWD/.python_deps python3 -m standard_vtol_nmpc.simulate
```

Pokretanje plota:

```bash
cd /home/imran/Repositories/px4-mpc
MPLCONFIGDIR=$PWD/.cache/matplotlib PYTHONPATH=$PWD/.python_deps python3 -m standard_vtol_nmpc.plot_results
```

Rezultati:

```text
standard_vtol_nmpc/results/transition_6dof_results.npz
standard_vtol_nmpc/results/transition_6dof.png
```

Sta treba vidjeti:

- `vx` ide prema `16 m/s`.
- `py` ostaje blizu nule.
- `pz` ostaje blizu nule uz mali prelazni overshoot.
- pitch se kratko poveca tokom tranzicije i vrati blizu nule.
- roll i yaw ostaju blizu nule u simetricnom scenariju.

### 3.2 Standard VTOL Faza 2: ROS/PX4 offboard

Ovo pokrece PX4/Gazebo standard VTOL model i NMPC offboard controller. Trebas
imati otvoren QGroundControl i Gazebo, jer ces tamo najlakse vidjeti da letjelica
stvarno poleti i krene naprijed.

Sta radi svaki novi node:

- `mpc_standard_vtol`: cita `/fmu/out/vehicle_status`,
  `/fmu/out/vehicle_attitude`, `/fmu/out/vehicle_angular_velocity` i
  `/fmu/out/vehicle_local_position`; rjesava CasADi NMPC; objavljuje
  `/fmu/in/offboard_control_mode` i `/fmu/in/vehicle_rates_setpoint`.
- `standard_vtol_reference`: objavljuje `nav_msgs/Odometry` referencu na
  `/px4_mpc/standard_vtol/reference`. Profil `transition` prvo drzi visinu,
  zatim rampira forward speed.
- `standard_vtol_commander`: objavljuje offboard heartbeat i salje PX4 komande
  za Offboard mode i arm.

Za prvi probni let koristi konzervativnije vrijednosti:

```bash
altitude:=12.0 forward_speed:=8.0
```

Kad vidis da je stabilno, mozes koristiti agresivniji transition:

```bash
altitude:=20.0 forward_speed:=16.0
```

#### 3.2.1 Build prije leta

Terminal za build:

```bash
cd /home/imran/Repositories/px4-mpc

source /opt/ros/jazzy/setup.bash

export PYTHONPATH=$PWD/.python_deps:$PYTHONPATH
export ACADOS_SOURCE_DIR=$PWD/acados
export LD_LIBRARY_PATH=$PWD/acados/lib:$LD_LIBRARY_PATH
export MPLCONFIGDIR=$PWD/.cache/matplotlib

colcon build --packages-up-to px4_mpc --symlink-install
source install/setup.bash
```

Provjeri da entry point-i postoje:

```bash
ros2 pkg executables px4_mpc | grep -E "mpc_standard_vtol|standard_vtol_reference|standard_vtol_commander"
```

#### 3.2.2 Terminal 1: PX4 standard VTOL SITL

U posebnom terminalu:

```bash
cd /home/imran/Repositories/PX4-Autopilot
make px4_sitl gz_standard_vtol
```

Sacekaj da Gazebo ucita standard VTOL model i da QGroundControl vidi vehicle.

#### 3.2.3 Terminal 2: DDS/Micro XRCE Agent

U posebnom terminalu:

```bash
cd /home/imran/Repositories/px4-mpc
export LD_LIBRARY_PATH=$PWD/microxrce_agent_install/lib:$LD_LIBRARY_PATH
$PWD/microxrce_agent_install/bin/MicroXRCEAgent udp4 -p 8888
```

Provjera da PX4 publish-a ROS 2 topic-e:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep fmu
```

Ako nema `/fmu/out/...` topic-a, PX4 i ROS 2 nisu spojeni. Prvo popravi agent
ili PX4 SITL prije pokretanja NMPC-a.

#### 3.2.4 Terminal 3: Standard VTOL NMPC + referenca

U posebnom terminalu:

```bash
cd /home/imran/Repositories/px4-mpc

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export PYTHONPATH=$PWD/.python_deps:$PYTHONPATH
export ACADOS_SOURCE_DIR=$PWD/acados
export LD_LIBRARY_PATH=$PWD/acados/lib:$LD_LIBRARY_PATH
export MPLCONFIGDIR=$PWD/.cache/matplotlib

ros2 launch px4_mpc mpc_standard_vtol_launch.py altitude:=12.0 forward_speed:=8.0 profile:=transition
```

Ovaj launch pokrece:

- `px4_mpc/mpc_standard_vtol`
- `px4_mpc/standard_vtol_reference`

Controller odmah salje offboard heartbeat i rate/thrust setpoint stream. PX4
Offboard mode obicno nece biti prihvacen ako ovaj stream jos ne dolazi.

Provjera u jos jednom terminalu, jednu po jednu komandu:

```bash
source /opt/ros/jazzy/setup.bash
source /home/imran/Repositories/px4-mpc/install/setup.bash
ros2 topic hz /fmu/in/offboard_control_mode
ros2 topic hz /fmu/in/vehicle_rates_setpoint
```

#### 3.2.5 Terminal 4: arm + Offboard mode

Kad controller radi par sekundi, u posebnom terminalu pokreni commander:

```bash
cd /home/imran/Repositories/px4-mpc

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run px4_mpc standard_vtol_commander
```

Commander ce:

1. slati offboard heartbeat,
2. traziti PX4 Offboard mode,
3. armati letjelicu.

Alternativa je da commander pokrenes automatski zajedno sa controllerom:

```bash
ros2 launch px4_mpc mpc_standard_vtol_launch.py altitude:=12.0 forward_speed:=8.0 profile:=transition auto_start:=true
```

Za prvi test je sigurnije pokrenuti commander rucno u Terminalu 4, jer jasnije
vidis trenutak kada saljes arm/offboard.

#### 3.2.6 Kako vidjeti da leti

U Gazebo-u treba vidjeti da standard VTOL:

- arma se,
- podigne se prema zadatoj visini,
- nakon kratkog delay-a krene naprijed po lokalnom `x` smjeru,
- nastavlja forward transition prema zadatom `forward_speed`.

U QGroundControl-u treba vidjeti:

- mode: `Offboard`,
- armed state: armed,
- altitude raste,
- vehicle se pomjera naprijed.

CLI provjere:

```bash
ros2 topic echo /fmu/out/vehicle_status --once
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic echo /px4_mpc/standard_vtol/reference --once
```

Za PX4 local position, `z` je NED down osa. To znaci:

- kad letjelica poleti, `/fmu/out/vehicle_local_position.z` ide negativno,
- kad krene naprijed, `/fmu/out/vehicle_local_position.x` treba rasti.

Kontinuirano gledanje pozicije:

```bash
ros2 topic echo /fmu/out/vehicle_local_position
```

Kontinuirano gledanje reference:

```bash
ros2 topic echo /px4_mpc/standard_vtol/reference
```

Ako koristis RViz, korisni topic-i su:

```text
/px4_mpc/standard_vtol/predicted_path
/px4_mpc/standard_vtol/reference_marker
```

#### 3.2.7 Korisni topic-i za standard VTOL offboard

```text
/fmu/out/vehicle_status
/fmu/out/vehicle_attitude
/fmu/out/vehicle_angular_velocity
/fmu/out/vehicle_local_position
/fmu/in/offboard_control_mode
/fmu/in/vehicle_rates_setpoint
/px4_mpc/standard_vtol/reference
/px4_mpc/standard_vtol/predicted_path
/px4_mpc/standard_vtol/reference_marker
```

Vazno: `ros2 launch px4_mpc mpc_quadrotor_launch.py` pokrece quadrotor
controller, ne standard VTOL controller. Za standard VTOL koristi
`mpc_standard_vtol_launch.py`.

#### 3.2.8 Ako ne poleti

Ako QGroundControl pise `Not Ready` ili `Health issues`, nemoj forsirati arm.
Novi `standard_vtol_commander` po defaultu ceka da PX4 objavi:

```text
pre_flight_checks_pass: true
failsafe: false
```

Provjeri PX4 status:

```bash
ros2 topic echo /fmu/out/vehicle_status --once
```

Bitna polja:

```text
pre_flight_checks_pass
failsafe
arming_state
nav_state
is_vtol
vehicle_type
```

Ako `pre_flight_checks_pass` nije `true`, pogledaj detaljnije:

```bash
ros2 topic echo /fmu/out/failsafe_flags --once
ros2 topic echo /fmu/out/health_report --once
```

Najcesci uzroci u SITL-u su:

- PX4/Gazebo jos nisu potpuno inicijalizovani,
- nema lokalne pozicije ili altitude estimate-a,
- home/global position jos nije validan,
- Offboard signal/setpoint stream ne dolazi dovoljno dugo,
- PX4 je vec usao u failsafe nakon prethodnog pokusaja.

U PX4 SITL shell-u korisno je probati:

```bash
commander status
commander check
```

Ako commander loguje `waiting for PX4 health`, to je namjerno: ceka da QGC/PX4
prestanu prijavljivati health problem. Nemoj gasiti tu zastitu osim za debug.
Za override postoji parametar, ali ga ne koristi za normalan let:

```bash
ros2 run px4_mpc standard_vtol_commander --ros-args -p require_preflight_checks:=false
```

Ako rotori samo kratko krenu pa stanu, to obicno znaci da je PX4 odbio arm,
izletio iz Offboard moda ili odmah disarmovao zbog failsafe-a. Gledaj ACK:

```bash
ros2 topic echo /fmu/out/vehicle_command_ack
```

Ako nema PX4 topic-a:

```bash
ros2 topic list | grep fmu
```

Ako nema topic-a, provjeri PX4 SITL i Micro XRCE Agent.

Ako Offboard mode nije prihvacen:

- prvo pokreni `mpc_standard_vtol_launch.py`,
- sacekaj 2-3 sekunde da heartbeat krene,
- tek onda pokreni `standard_vtol_commander`,
- provjeri:

```bash
ros2 topic hz /fmu/in/offboard_control_mode
ros2 topic hz /fmu/in/vehicle_rates_setpoint
```

Ako node padne na `ModuleNotFoundError: casadi`, terminal nema lokalne Python
dependency-je:

```bash
export PYTHONPATH=/home/imran/Repositories/px4-mpc/.python_deps:$PYTHONPATH
```

Ako node ceka stanje i ne salje komande, provjeri da dolaze:

```bash
ros2 topic echo /fmu/out/vehicle_attitude --once
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic echo /fmu/out/vehicle_angular_velocity --once
```

Ako letjelica krene lose:

- u QGroundControl prebaci iz `Offboard` u `Hold` ili `Position`,
- klikni `Land` ili `RTL`,
- ili u PX4 SITL shell-u:

```bash
commander mode hold
commander land
commander disarm
```

## 4. Brze debug komande

Lista paketa:

```bash
ros2 pkg list | grep -E "px4_mpc|mpc_msgs|px4_msgs|px4_offboard"
```

Lista PX4 topica:

```bash
ros2 topic list | grep fmu
```

Provjera da acados Python radi:

```bash
PYTHONPATH=$PWD/.python_deps ACADOS_SOURCE_DIR=$PWD/acados python3 -c "import casadi, acados_template; print(casadi.__version__); print(acados_template.__file__)"
```

Provjera da CasADi offline standard VTOL dio radi:

```bash
PYTHONPATH=$PWD/.python_deps python3 -m py_compile standard_vtol_nmpc/model.py standard_vtol_nmpc/mpc_casadi.py standard_vtol_nmpc/simulate.py standard_vtol_nmpc/plot_results.py
```

Ako ROS launch pokusava pisati u `~/.ros` a okruzenje to blokira, koristi:

```bash
mkdir -p .ros/log
export ROS_LOG_DIR=$PWD/.ros/log
```
