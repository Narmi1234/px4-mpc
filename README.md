# px4-mpc
This package contains an MPC integrated with with [PX4 Autopilot](https://px4.io/) and [ROS 2](https://ros.org/).

The MPC uses the [acados framework](https://github.com/acados/acados)

## Standard VTOL hover-to-forward-flight rad

Standard VTOL NMPC je validiran do L4b authority-transfer gatea pri približno
12.27 m/s. Puna VTOL tranzicija nije dokazana: L4c je ugasio lift motore, ali je
pao zbog longitudinalne/vertikalne nestabilnosti. Za trenutno dokazano stanje,
ograničenja i poštenu ocjenu PhD doprinosa prvo čitaj:

1. [`STANDARD_VTOL_NMPC_FINAL_SUMMARY.md`](STANDARD_VTOL_NMPC_FINAL_SUMMARY.md)
   — autoritativni završni presjek: model, jednačine, porijeklo parametara,
   NMPC/PX4 podjela, rezultati, posljednji uspješni demo, ograničenja i odluka.
2. [`STANDARD_VTOL_PROFESSOR_DEMO.md`](STANDARD_VTOL_PROFESSOR_DEMO.md) —
   kraći demonstracijski materijal; za aktuelni live demo koristiti L4b komandu
   iz završnog presjeka.
3. [`STANDARD_VTOL_PHD_PROGRESS_REPORT.md`](STANDARD_VTOL_PHD_PROGRESS_REPORT.md)
   — samostalan presjek za mentorski sastanak: cilj, model, porijeklo
   parametara, PX4 patch, rezultati, ograničenja i naredne odluke.
4. [`STANDARD_VTOL_ROBUST_TRANSITION_RUNBOOK.md`](STANDARD_VTOL_ROBUST_TRANSITION_RUNBOOK.md)
   — aktivne komande i R0–R4 gateovi novog 16-state NMPC pristupa.

5. [`STANDARD_VTOL_ROBUST_NMPC_ARCHITECTURE.md`](STANDARD_VTOL_ROBUST_NMPC_ARCHITECTURE.md)
   — aktivna arhitektura, tačna podjela NMPC/PX4 odgovornosti, jednačine i
   faze do pune NMPC tranzicije.
6. [`STANDARD_VTOL_RATE_IDENTIFICATION.md`](STANDARD_VTOL_RATE_IDENTIFICATION.md)
   — aktivna Faza 1, train/validation rezultati i razlog prelaska sa
   first-order laga na torque-informed 16-state rotacijski model.
7. [`STANDARD_VTOL_NMPC_ROADMAP.md`](STANDARD_VTOL_NMPC_ROADMAP.md) — historija
   prihvaćenih gateova i razlog povlačenja starog Gate D pristupa.
8. [`STANDARD_VTOL_PUSHER_FORWARD_RUNBOOK.md`](STANDARD_VTOL_PUSHER_FORWARD_RUNBOOK.md)
   — zaključani Gate A postupak i prihvaćeni rezultat.
9. [`STANDARD_VTOL_GATE_B_RUNBOOK.md`](STANDARD_VTOL_GATE_B_RUNBOOK.md) —
   prihvaćeni odvojeni 5 m/s i 8 m/s MC gateovi.
10. [`STANDARD_VTOL_GATE_C_RUNBOOK.md`](STANDARD_VTOL_GATE_C_RUNBOOK.md) —
   prihvaćeni stock PX4 front/back transition uz NMPC shadow računanje.
11. [`STANDARD_VTOL_GATE_D_RUNBOOK.md`](STANDARD_VTOL_GATE_D_RUNBOOK.md) —
   arhivirani eksperimentalni postupak; nije dozvoljen za novi let.

Pozadinski dokumenti su:

1. [`STANDARD_VTOL_PLANT_VALIDATION.md`](STANDARD_VTOL_PLANT_VALIDATION.md) —
   tačne PX4/QGroundControl komande, testni let, ULog i poređenje s modelom;
2. [`STANDARD_VTOL_TRANSITION_NMPC.md`](STANDARD_VTOL_TRANSITION_NMPC.md) —
   jednačine, razlika između punog 18-state planta i 10-state NMPC modela, te
   kompletan implementation path.
3. [`STANDARD_VTOL_REIDENTIFICATION.md`](STANDARD_VTOL_REIDENTIFICATION.md) —
   train/validation podjela ULogova, dodatni letovi na 12 i 18 m/s i fit
   reduciranih aerodinamičkih koeficijenata.
4. [`STANDARD_VTOL_TRIM_CORRIDOR.md`](STANDARD_VTOL_TRIM_CORRIDOR.md) — kako se
   ravnotežne tačke računaju i kako regenerisati 0–22 m/s corridor.
5. [`STANDARD_VTOL_OFFBOARD_RUNBOOK.md`](STANDARD_VTOL_OFFBOARD_RUNBOOK.md) —
   tačan postupak za zaštićeni hover-only Offboard regression test;
6. [`STANDARD_VTOL_HOVER_RESULTS.md`](STANDARD_VTOL_HOVER_RESULTS.md) — dokazani
   hover rezultati, aktivne zaštite i granica tog checkpointa;
7. [`STANDARD_VTOL_MC_FORWARD_RUNBOOK.md`](STANDARD_VTOL_MC_FORWARD_RUNBOOK.md)
   — ponovljivi 2 m/s gate u MC režimu, bez tranzicije i bez pushera;
8. [`STANDARD_VTOL_MC_FORWARD_RESULTS.md`](STANDARD_VTOL_MC_FORWARD_RESULTS.md)
   — prihvaćeni live rezultat i tačna granica sljedećeg implementacijskog
   koraka;
9. [`STANDARD_VTOL_EXTERNAL_PUSHER_RUNBOOK.md`](STANDARD_VTOL_EXTERNAL_PUSHER_RUNBOOK.md)
   — custom PX4 branch i dokazani NMPC `0 -> 0.05 -> 0` live gate.

Za plant validaciju se ne koristi Offboard i ne šalju se direktne motorne
komande. PX4 upravlja Gazebo letjelicom, a `tools/validate_standard_vtol_ulog.py`
nakon leta samo čita ULog.

Trenutna sigurna provjera (bez PX4-a i bez novog leta) je:

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
PYTHONPATH="px4_mpc:.venv/lib/python3.12/site-packages:${PYTHONPATH}" \
  /usr/bin/python3 -m pytest -q px4_mpc/test
```

![px4-mpc](https://github.com/user-attachments/assets/6713b8e6-815f-42fe-b3a0-51708d3416e5)

## Paper
If you find this package useful in an academic context, please consider citing the paper

- Roque, Pedro, Sujet Phodapol, Elias Krantz, Jaeyoung Lim, Joris Verhagen, Frank Jiang, David Dorner, Roland Siegwart, Ivan Stenius, Gunnar Tibert, Huina Mao, Jana Tumova, Christer Fuglesang, Divos V. Dimarogonas. "Towards Open-Source and Modular Space Systems with ATMOS." arXiv preprint arXiv:2501.16973 (2025).
. [[preprint](https://arxiv.org/abs/2501.16973)]

```
@article{roque2025towards,
  title={Towards Open-Source and Modular Space Systems with ATMOS},
  author={Roque, Pedro and Phodapol, Sujet and Krantz, Elias and Lim, Jaeyoung and Verhagen, Joris and Jiang, Frank and Dorner, David and Siegwart, Roland and Stenius, Ivan and Tibert, Gunnar and others},
  journal={arXiv preprint arXiv:2501.16973},
  year={2025}
}
```

## Setup
The MPC formulation uses acados. In order to install acados, follow the following [instructions](https://docs.acados.org/installation/)
To build the code, clone the following repositories into a ros2 workspace
Dependencies
- [px4_msgs](https://github.com/PX4/px4_msgs/pull/15)
- [px4-offboard](https://github.com/Jaeyoung-Lim/px4-offboard) (Optional): Used for RViz visualization

```
colcon build --packages-up-to px4_mpc
```

### Testing demos
```
ros2 run px4_mpc quadrotor_demo
```

### Running MPC with PX4 SITL
In order to run the SITL(Software-In-The-Loop) simulation, the PX4 simulation environment and ROS2 needs to be setup.
For instructions, follow the [documentation](https://docs.px4.io/main/en/ros/ros2_comm.html)

Run PX4 SITL
```
make px4_sitl gazebo
```

Run the micro-ros-agent
```
micro-ros-agent udp4 --port 8888
```

In order to launch the mpc quadrotor in a ros2 launchfile,
```
ros2 launch px4_mpc mpc_quadrotor_launch.py 
```

The mpc_spacecraft_launch.py file includes optional arguments:

- **mode**: Control mode (wrench by default). Options: wrench, rate, direct_allocation.  
- **namespace**: Spacecraft namespace ('' by default).  
- **setpoint_from_rviz**: Use RViz for setpoints (True by default).

**Example:**
```bash
ros2 launch px4_mpc mpc_spacecraft_launch.py mode:=wrench namespace:=<namespace> setpoint_from_rviz:=False
```
