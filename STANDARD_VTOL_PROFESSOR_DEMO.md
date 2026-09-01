# Standard VTOL NMPC — demonstracija za konsultacije

Ova demonstracija pokazuje dvije odvojene, već implementirane stvari:

1. **offline NMPC tranziciju** od hovera do 15 m/s, uključujući optimizirani
   pusher, body-rate reference i `lambda` allocation;
2. **live SITL L1 authority transfer**: NMPC ubrzava letjelicu do 5.312 m/s,
   smanjuje `lambda` sa 1.0 na 0.8, zatim usporava i vraća `lambda=1`.

Ne tvrditi da je puna live tranzicija završena. Trenutno je potvrđen offline
transition OCP i prvi live prijenos 20% MC autoriteta prema FW/aerodinamičkom
sloju. Lift motori tokom L1 ostaju uključeni; sljedeći eksperiment je L2 sa
`lambda>=0.5` i brzinom 8–10 m/s.

L1 gate je zatim prošao i live SITL: 29.51 s Offboarda, 5.312 m/s,
`lambda_min=0.800`, pusher 0.155, max greška visine 0.376 m, cross-track
0.302 m i nula solver failurea. Završio je na praktično nultoj brzini i
`lambda=1`. Postupak i puni status su u R4-L1 dijelu
`STANDARD_VTOL_ROBUST_TRANSITION_RUNBOOK.md`.

## 1. Najbrži demo bez Gazeba

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_robust_transition_offline_gate.bash
```

Završetak mora biti:

```text
ROBUST_TRANSITION_MATRIX=PASS
```

Nominalni rezultat je približno 15 m/s, manje od 0.1 m maksimalne greške
visine i `lambda` blizu nule na kraju. Matrica uključuje nominalni i četiri
disturbance slučaja. Ovo je simulacija zatvorene petlje vlastitog 16-state
planta, a nije Gazebo let.

## 2. Glavni live demo — L1 authority transfer

Koristi samo custom PX4 branch `nmpc-external-pusher`. Prije pokretanja:

```bash
cd /home/imran/Repositories/PX4-Autopilot
git branch --show-current
```

Mora ispisati `nmpc-external-pusher`.

### Terminal 1 — patched PX4 i Gazebo

Deaktivirati Python venv ako je aktivan, zatim:

```bash
cd /home/imran/Repositories/PX4-Autopilot
deactivate 2>/dev/null || true
make px4_sitl gz_standard_vtol
```

U PX4 konzoli:

```text
param set VT_EXT_PUSH_EN 1
param set VT_EXT_PUSH_MAX 0.25
param set VT_EXT_PUSH_SLEW 0.10
param set VT_EXT_ALLOC_EN 1
param set VT_EXT_AL_SLEW 0.10
param show VT_EXT_PUSH_EN
param show VT_EXT_PUSH_MAX
param show VT_EXT_PUSH_SLEW
param show VT_EXT_ALLOC_EN
param show VT_EXT_AL_SLEW
```

Očekivano je redom `1`, `0.25`, `0.10`, `1`, `0.10`.

### Terminal 2 — DDS agent

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
"${MICRO_XRCE_AGENT_DIR}/bin/MicroXRCEAgent" udp4 -p 8888
```

### Terminal 3 — L1 NMPC node

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
ros2 launch px4_mpc standard_vtol_robust_l1_gate_launch.py
```

Mora pisati `guarded L1 allocation mode (5 m/s, lambda >= 0.8, MC only)`.
U QGC armati, poletjeti u **Position** modu na 8–10 m, usmjeriti letjelicu
prema slobodnom prostoru i potpuno je smiriti. Ne komandovati PX4 tranziciju.

### Terminal 4 — automatski gate

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_robust_allocation_l1_gate.bash
```

Upisati `YES` samo dok je letjelica stabilna. Završna linija mora biti:

```text
ROBUST_ALLOCATION_L1=PASS
```

Test traje oko 30 s. NMPC komanduje profil `0 -> 5 -> 0 m/s`, pusher i
`lambda: 1 -> 0.8 -> 1`, dok PX4 ostaje u MC režimu. Prihvaćeni rezultat je:

```text
last_offboard_duration=29.51 s
solver_failures=0
solve_time_p99=25.07 ms
abort_reason=allocation_l1_test_timeout
ever_active=True, ever_valid=True
max_forward_speed=5.312 m/s
max_cross_track=0.302 m
max_altitude_error=0.376 m
max_pusher=0.155
min_lambda=0.800
final_forward_speed=-0.007 m/s
final_applied_lambda=1.000
```

`active=False` u završnom statusu je očekivano jer je node već vratio PX4 u
Position. `ever_active=True` dokazuje da je allocation kanal bio aktivan tokom
leta. Prihvaćeni ULog je:

```text
validation_logs/accepted/robust_allocation_l1_pass_2026-09-01.ulg
```

Ako nema vremena za novi let, profesoru pokazati ovaj rezultat, završni status
u R4-L1 runbooku i offline matricu iz prvog dijela. Stari pet-sekundni hover
je samo prethodni R3b checkpoint, a nije glavna demonstracija.

## 3. Šta tačno pokazati u kodu

- jednačine planta:
  `px4_mpc/px4_mpc/models/standard_vtol_robust_casadi_model.py`;
- cost, constraints, horizont i acados OCP:
  `px4_mpc/px4_mpc/controllers/standard_vtol_robust_nmpc.py`;
- receding-horizon ROS petlja i prvi primijenjeni control sample:
  `px4_mpc/px4_mpc/standard_vtol_robust_shadow_node.py`;
- PX4 primjena `lambda`:
  `PX4-Autopilot/src/modules/vtol_att_control/standard.cpp`;
- ROS/PX4 ugovor:
  `px4_msgs/msg/VtolNmpcAllocationSetpoint.msg` i
  `VtolNmpcAllocationStatus.msg`.

NMPC ulaz je

```math
u=[c_{lift},c_{push},p_{sp},q_{sp},r_{sp},\lambda].
```

Patched PX4 primjenjuje

```math
T_{lift}=\lambda T_{MC},\qquad
\tau_{MC}=\lambda\tau_{MC,PID},\qquad
\tau_{FW}=(1-\lambda)\tau_{FW,PID}.
```

NMPC bira sporu tranzicijsku putanju i allocation. PX4 zadržava estimator,
brze body-rate petlje, mapiranje na pojedinačne motore/serva i failsafe.

## 4. Pošten zaključak za mentora

> Identificiran je grey-box model iz SDF-a i ULogova, implementiran je
> 16-state nonlinear constrained OCP i prošla je offline disturbance matrica.
> Live SITL potvrđuje real-time NMPC, hover kontrolu i prvi authority transfer
> `lambda: 1 -> 0.8 -> 1` uz pusher i povratak brzine na nulu. Puna live
> tranzicija još nije rezultat; naredni gate produbljuje transfer prema
> `lambda=0.5` uz veći wing-borne doprinos i iste recovery zaštite.
