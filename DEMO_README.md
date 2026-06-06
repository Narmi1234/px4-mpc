# px4-mpc Local Demo Notes

Ovaj fajl je prakticni podsjetnik za lokalni setup, build i demo pokretanje
`px4-mpc` workspace-a na ovoj masini.

## 1. Sta je u workspace-u

Workspace root:

```bash
/home/imran/Repositories/px4-mpc
```

Lokalno su dodani dependency source folderi:

```bash
px4_msgs/
px4-offboard/
acados/
.python_deps/
```

Nemoj ih commitati u upstream repo ako ne zelis vendored dependency-je u svom
git history-ju.

## 2. Jednokratni system setup

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

## 3. Source dependency-ji

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

## 4. Environment za svaki novi terminal

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

Ako `install/setup.bash` jos ne postoji, prvo uradi build iz sekcije 5.

## 5. Build workspace-a

```bash
cd /home/imran/Repositories/px4-mpc
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to px4_mpc
source install/setup.bash
```

Prvi build `px4_msgs` paketa moze trajati vise minuta jer generise veliki broj
ROS 2 poruka.

## 6. Pokretanje samo MPC node-a

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

Napomena: originalni README spominje `quadrotor_demo`, ali u ovom repo-u stvarni
entry point je `mpc_quadrotor`.

## 7. Pokretanje RViz demo launch-a

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

```bash
/fmu/out/vehicle_status
/fmu/out/vehicle_attitude
/fmu/out/vehicle_local_position
```

Provjera:

```bash
ros2 topic list | grep fmu
```

Ako nema `/fmu/out/...` topica, PX4 nije spojen na ROS 2.

## 8. Puni PX4 SITL demo

Za puni demo trebaju tri terminala.

Terminal 1: PX4 SITL

```bash
cd <PX4-Autopilot>
make px4_sitl gazebo
```

Terminal 2: DDS/micro agent

Na ovoj masini je agent buildan lokalno iz source-a u:

```bash
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

Terminal 3: MPC + RViz

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

## 9. Sta treba vidjeti u demo-u

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

## 10. Ko pokrece dron i ko daje trajektoriju

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

## 11. Sigurnosna napomena za demo

Kad PX4 udje u `Offboard`, `px4_mpc` salje body-rate/thrust komande na:

```bash
/fmu/in/vehicle_rates_setpoint
```

Ovaj controller nije "sigurni demo autopilot". Ako frame transformacije,
altitude znak, thrust mapping ili tuning ne odgovaraju SITL-u, dron moze
krenuti u neocekivanom smjeru. Drzi QGroundControl spreman za prebacivanje
nazad iz `Offboard` moda.

Ako dron krene nekontrolisano:

U QGroundControl prebaci iz `Offboard` u `Hold` ili `Position`, ili klikni
`Land`/`RTL`.

Iz PX4 SITL shell-a:

```bash
commander mode hold
```

ili:

```bash
commander land
```

Kao zadnja opcija:

```bash
commander disarm
```

## 12. Brze debug komande

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

Ako ROS launch pokusava pisati u `~/.ros` a okruzenje to blokira, koristi:

```bash
mkdir -p .ros/log
export ROS_LOG_DIR=$PWD/.ros/log
```
