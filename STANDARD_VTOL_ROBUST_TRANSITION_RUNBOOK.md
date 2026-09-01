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

Za cijeli prihvaćeni R1/R2 matrix dovoljna je jedna naredba:

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_robust_transition_offline_gate.bash
```

Završna linija mora biti `ROBUST_TRANSITION_MATRIX=PASS`.

Pojedinačni nominalni slučaj se pokreće ovako:

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

### R3a — prvi read-only hover test

Ovaj test još ne radi tranziciju. Dokazuje ROS/DDS state mapping, 20 Hz solve i
da novi node nema PX4 output publisher.

Terminal 1 — PX4/Gazebo:

```bash
cd /home/imran/Repositories/PX4-Autopilot
make px4_sitl gz_standard_vtol
```

Terminal 2 — Micro XRCE Agent:

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
"${MICRO_XRCE_AGENT_DIR}/bin/MicroXRCEAgent" udp4 -p 8888
```

Terminal 3 — novi read-only shadow node:

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
ros2 launch px4_mpc standard_vtol_robust_shadow_launch.py
```

Mora ispisati:

```text
Robust Standard VTOL NMPC started in read-only shadow mode;
no /fmu/in publisher exists in this node
```

U QGC armati letjelicu, poletjeti u **Position** modu na 5–7 m i sačekati da
se vertikalna brzina smiri. Ne uključivati Offboard i ne raditi tranziciju.

Terminal 4:

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_robust_hover_shadow_gate.bash
```

Očekivana završna linija:

```text
ROBUST_HOVER_SHADOW=PASS
```

Status mora sadržavati `read_only=True`, `publishes_fmu=False`,
`solver_failures=0` i `solve_time_p99<40 ms`. Ako capture vrati
`vehicle_not_armed` ili `odometry_stale`, ne pokušavati Offboard; popraviti
PX4/DDS stanje i ponoviti samo R3a.

Aktivna real-time konfiguracija koristi 20 shooting intervala na horizontu od
2.0 s. Cijela offline matrica je ponovo prošla s tom konfiguracijom. Prvih 100
solveova poslije capturea su warm-up i ne ulaze u p99 statistiku; R3b skripta
čeka ukupno osam sekundi prije nego što uopšte dozvoli output.

### Prihvaćeni R3a rezultat — 2026-08-31

```text
ROBUST_HOVER_SHADOW=PASS
read_only=True
publishes_fmu=False
armed=True, nav_state=4 (Position)
state_age=0.012 s
solver_status=0
solver_failures=0
warmup_remaining=0
solve_time=17.21 ms
solve_time_p99=30.43 ms
control=[0.4560, 0.0001, -0.0733, 0.0320, 0.0232, 0.9889]
```

Jedan međustatus je imao p99 `40.96 ms`, ali nije bilo solver failurea, a
završni steady-state prozor je ostao ispod 40 ms. R3a potvrđuje komunikaciju i
računanje, ne validira još primijenjenu NMPC komandu.

Naredni gate R3b je guarded robust-NMPC hover output sa zaključanim
`pusher=0` i `lambda=1`. Tek nakon R3b PASS-a smije početi L1 allocation test.

### R3b — guarded pet-sekundni robust-NMPC hover

R3b prvi put primjenjuje komande novog 16-state kontrolera. Pusher je fizički
zaključan na nulu, `lambda` je zaključana na jedan, collective koristi već
validirani vertikalni safety loop, a rate komande prolaze MC bounds i slew
limiter. Timeout ili safety prekršaj traži PX4 Position mode.

Pokrenuti PX4/Gazebo i Micro XRCE Agent kao za R3a. Ugasiti R3a node ako još
radi. Terminal 3:

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
ros2 launch px4_mpc standard_vtol_robust_hover_gate_launch.py
```

Mora pisati `guarded 5 s hover mode`. U QGC armati, poletjeti u Position modu
na 5–7 m i potpuno smiriti letjelicu. Terminal 4:

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_robust_hover_output_gate.bash
```

Nakon provjere upisati `YES`. Držati QGC spreman za ručni Position fallback.
Prije leta u PX4 konzoli mora biti:

```text
param set VT_EXT_ALLOC_EN 1
param set VT_EXT_AL_SLEW 0.10
```

Očekivana završna linija je:

```text
ROBUST_HOVER_OUTPUT=PASS (including explicit PX4 allocation channel)
```

PASS zahtijeva `abort_reason=robust_hover_test_timeout`, povratak iz Offboarda,
nula solver failurea, ugašen output te PX4 allocation status
`ever_active=True,ever_valid=True`. Svaki drugi abort je FAIL; sletjeti i ne
ponavljati prije analize statusa.

### Prihvaćeni R3b rezultat — 2026-08-31

```text
ROBUST_HOVER_OUTPUT=PASS
publishes_fmu=True
output_requested=False, offboard=False
armed=True, nav_state=2 (Position poslije automatskog fallbacka)
state_age=0.000 s
solver_status=0, solver_failures=0
solve_time=14.98 ms, solve_time_p99=26.97 ms
last_offboard_duration=5.05 s
abort_reason=robust_hover_test_timeout
published_control=[0.5200, 0.0000, -0.0098, -0.0097, 0.0155, 1.0000]
```

R3b potvrđuje prvi primijenjeni izlaz 16-state robustnog NMPC-a, stabilan
pet-sekundni hover i automatski povratak u Position. Ne potvrđuje tranzicijsku
raspodjelu: tokom cijelog leta `pusher=0` i `lambda=1`.

Aktivni PX4 branch `nmpc-external-pusher` sada ima eksplicitni NMPC `lambda`
ulaz i status povratnu poruku; ROS i PX4 build su prošli. Prihvaćeni rezultat
iznad prethodi tom kanalu, pa se R3b jednom ponavlja s novim PASS uslovom.
Tek nakon tog handshake dokaza slijedi bench `lambda=0.8`, pa L1 offline,
shadow i live.

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

### R4-L1 — aktivni eksperiment

L1 je prvi stvarni prijenos autoriteta. Ne šalje PX4 transition komandu i
letjelica mora ostati MC. Profil je `0 -> 5 -> 0 m/s`, pusher je ograničen na
0.25, a NMPC allocation na `lambda >= 0.8`. Collective se kompenzira sa
`c_hover/lambda`, tako da 20% authority transfer ne znači 20% gubitka lifta.

Offline prerequisite je potvrđen sa `solver_failures=0`, završnom brzinom
4.998 m/s, max greškom visine 0.016 m i završnim `lambda=0.834`:

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_robust_allocation_l1_offline_gate.bash
```

Mora završiti sa `ROBUST_ALLOCATION_L1_OFFLINE=PASS`.

Terminal 1 — patched PX4/Gazebo:

```bash
cd /home/imran/Repositories/PX4-Autopilot
deactivate 2>/dev/null || true
make px4_sitl gz_standard_vtol
```

U PX4 konzoli postaviti i provjeriti:

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

Terminal 2 — DDS agent:

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
"${MICRO_XRCE_AGENT_DIR}/bin/MicroXRCEAgent" udp4 -p 8888
```

Terminal 3 — L1 node:

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
ros2 launch px4_mpc standard_vtol_robust_l1_gate_launch.py
```

Mora ispisati `guarded L1 allocation mode`. U QGC poletjeti u Position modu
na 8–10 m, poravnati pravac prema slobodnom prostoru i potpuno smiriti let.

Terminal 4:

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_robust_allocation_l1_gate.bash
```

Upisati `YES`. Tokom oko 30 s posmatrati brzinu, visinu i pravac. Ne pritiskati
VTOL transition. PASS zahtijeva automatski Position fallback, bez solver
failurea, aktivan allocation handshake i primijenjeni minimum `lambda=0.8x`:

```text
ROBUST_ALLOCATION_L1=PASS
```

Svaki drugi završetak je FAIL: ostati u Position, sletjeti i poslati završni
status prije ponavljanja.

### Prihvaćeni R4-L1 rezultat — 2026-09-01

```text
ROBUST_ALLOCATION_L1=PASS
last_offboard_duration=29.51 s
solver_failures=0
solve_time=14.22 ms, solve_time_p99=25.07 ms
abort_reason=allocation_l1_test_timeout
allocation: ever_active=True, ever_valid=True
max_forward_speed=5.312 m/s
max_cross_track=0.302 m
max_altitude_error=0.376 m
max_pusher=0.155
min_lambda=0.800
final_forward_speed=-0.007 m/s
final_applied_lambda=1.000
```

Ovo potvrđuje prvi **live NMPC-owned authority transfer**. NMPC je kroz novi
DDS/PX4 kanal smanjio MC allocation na 80%, koristio pusher, dostigao 5.312
m/s, vratio brzinu praktično na nulu i obnovio `lambda=1`, dok je letjelica
ostala u MC režimu. `active=False` u završnom statusu je očekivano jer je PX4
već vraćen iz Offboarda u Position; `ever_active=True` dokazuje da je kanal
bio aktivan tokom testa.

Ovaj rezultat ne predstavlja punu front transition: lift motori nisu ugašeni
i `lambda` nije išla ispod 0.8. On je prihvaćeni eksperimentalni checkpoint
prije L2 (`lambda>=0.5`, veća brzina).
