# Standard VTOL NMPC — demonstracija za konsultacije

Ova demonstracija pokazuje dvije odvojene, već implementirane stvari:

1. **offline NMPC tranziciju** od hovera do 15 m/s, uključujući optimizirani
   pusher, body-rate reference i `lambda` allocation;
2. **live SITL NMPC izvršenje** u sigurnom pet-sekundnom hoveru, uključujući
   novi eksplicitni `lambda` kanal do patched PX4-a.

Ne tvrditi da je puna live tranzicija završena. Trenutno je potvrđen offline
transition OCP i live hover/data/allocation lanac. Sljedeći eksperiment je
ograničeni `lambda: 1.0 -> 0.8 -> 1.0` gate.

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

## 2. Live SITL demo kompletnog interfejsa

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
param set VT_EXT_ALLOC_EN 1
param set VT_EXT_AL_SLEW 0.10
param show VT_EXT_ALLOC_EN
param show VT_EXT_AL_SLEW
```

Očekivano je `1` i `0.10`.

### Terminal 2 — DDS agent

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
"${MICRO_XRCE_AGENT_DIR}/bin/MicroXRCEAgent" udp4 -p 8888
```

### Terminal 3 — guarded NMPC node

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
ros2 launch px4_mpc standard_vtol_robust_hover_gate_launch.py
```

Mora pisati `guarded 5 s hover mode`. U QGC armati, poletjeti u **Position**
modu na 5–7 m i potpuno smiriti letjelicu. Ne komandovati tranziciju.

### Terminal 4 — automatski gate

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_robust_hover_output_gate.bash
```

Upisati `YES` samo dok je letjelica stabilna. Završna linija mora biti:

```text
ROBUST_HOVER_OUTPUT=PASS (including explicit PX4 allocation channel)
```

Status mora sadržavati:

```text
last_offboard_duration=oko 5 s
solver_failures=0
abort_reason=robust_hover_test_timeout
allocation=[requested=1.000,applied=1.000,...ever_active=True,ever_valid=True]
```

`ever_active=True` je dokaz da vrijednost nije ostala samo ROS poruka: PX4 ju
je prihvatio dok je bio armed, Offboard i body-rate control aktivan. Nakon pet
sekundi node automatski traži Position mode.

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
> Live SITL je potvrdio real-time solve, pet-sekundni NMPC hover i eksplicitni
> NMPC-to-PX4 allocation kanal. Puna live tranzicija još nije rezultat; naredni
> eksperiment postepeno smanjuje `lambda` uz pusher i stroge recovery gateove.

