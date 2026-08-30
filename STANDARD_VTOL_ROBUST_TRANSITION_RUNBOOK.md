# Standard VTOL robust transition — naredni gateovi

Ovo je aktivni dokument poslije neuspješnog starog Gate D pristupa. Stari
`scripts/run_transition_gate_d.bash` se više ne koristi za razvoj pune
tranzicije.

## Trenutni presjek

Novi lanac je odvojen od live ROS nodea:

```text
ULog run01/run03 train, run02/run04 validation
  -> stable LPV pitch-rate residual
  -> 16-state CasADi plant
  -> 6-input acados OCP
  -> offline closed-loop transition
```

NMPC ulazi su:

```text
[collective, pusher, p_sp, q_sp, r_sp, lambda]
```

`lambda=1` znači puni lift/MC autoritet, a `lambda=0` potpuno wing-borne
upravljanje. PX4 u konačnoj arhitekturi izvršava body-rate inner loop,
actuator allocation, estimaciju i failsafe, ali ne smije sam birati transition
schedule.

## Gate R0 — lokalni testovi

```bash
cd /home/imran/Repositories/px4-mpc
source .venv/bin/activate
PYTHONPATH=px4_mpc python -m unittest discover \
  -s px4_mpc/test -p 'test_standard_vtol_*model.py'
```

Očekivano: svi testovi `OK`.

## Gate R1 — nominalni offline front transition

```bash
cd /home/imran/Repositories/px4-mpc
source .venv/bin/activate
source scripts/setup_standard_vtol_nmpc.bash
PYTHONPATH="${PWD}/px4_mpc:${PYTHONPATH}" python \
  tools/simulate_standard_vtol_robust_transition.py
```

Očekivano:

```text
solver_failures=0
max_altitude_error_m < 2
max_abs_pitch_deg < 22
final_forward_speed_m_s >= 12.75
final_lambda <= 0.10
robust_transition_offline=PASS
```

Rezultat ovog checkpointa: 15.000 m/s, altitude error 0.081 m,
`lambda=0.000017`, p99 solve 7.52 ms.

## Gate R2 — disturbance matrica

Procijenjeni blend disturbance:

```bash
PYTHONPATH="${PWD}/px4_mpc:${PYTHONPATH}" python \
  tools/simulate_standard_vtol_robust_transition.py \
  --pitch-disturbance 0.235 \
  --output results/standard_vtol_robust_transition/plus_0235

PYTHONPATH="${PWD}/px4_mpc:${PYTHONPATH}" python \
  tools/simulate_standard_vtol_robust_transition.py \
  --pitch-disturbance -0.235 \
  --output results/standard_vtol_robust_transition/minus_0235
```

Nepoznati disturbance, koji OCP ne dobija kao parametar:

```bash
PYTHONPATH="${PWD}/px4_mpc:${PYTHONPATH}" python \
  tools/simulate_standard_vtol_robust_transition.py \
  --pitch-disturbance 0.10 --unmodeled-disturbance \
  --output results/standard_vtol_robust_transition/unmodeled_plus_010

PYTHONPATH="${PWD}/px4_mpc:${PYTHONPATH}" python \
  tools/simulate_standard_vtol_robust_transition.py \
  --pitch-disturbance -0.10 --unmodeled-disturbance \
  --output results/standard_vtol_robust_transition/unmodeled_minus_010
```

Sva četiri scenarija trenutno prolaze. Ekstrem `+0.47 rad/s²` ne prolazi i ne
smije se predstavljati kao riješena robusnost.

## Gate R3 — Gazebo shadow, naredni posao

Novi controller se pokreće bez objave actuator/offboard komandi. Iz PX4 se
uzimaju position, velocity, quaternion, body rates i airspeed; surface states
se propagiraju internim observerom. Snimaju se:

- jedna-step i 0.5 s predikcija brzine, pitch-ratea i visine;
- NMPC `collective/pusher/rate/lambda`, ali samo u shadow topic;
- solver status i p99 vrijeme;
- applied PX4 actuator/allocation telemetry.

Acceptance prije prvog live allocation testa:

```text
solver failures = 0
solver p99 < 40 ms
0.5 s pitch-rate RMSE <= 0.10 rad/s
0.5 s vertical-speed RMSE <= 0.50 m/s
bez NaN, stale state ili constraint violation
```

## Gate R4 — prvi live allocation test

Tek nakon R3 PASS-a koristi se PX4 offboard-rate/allocation branch. Prvi let
nije puna tranzicija:

```text
L1: lambda 1.0 -> 0.8 -> 1.0, airspeed do 5 m/s
L2: lambda 1.0 -> 0.5 -> 1.0, airspeed 8–10 m/s
L3: lambda 1.0 -> 0.2 -> 1.0, airspeed 11–13 m/s
L4: lambda 1.0 -> 0.0 -> 1.0, puna front/back tranzicija
```

Svaki gate mora prvo proći offline, zatim shadow, zatim jedan live SITL let.
QGC Position ostaje ručni recovery. Gate se ne ponavlja nakon aborta prije
pregleda statusa i ULoga.
