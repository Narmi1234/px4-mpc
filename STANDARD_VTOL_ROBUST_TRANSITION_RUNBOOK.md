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

### R4-L2 — 9 m/s i 50% authority transfer

L2 ostaje u MC stanju, ali prvi put traži značajan wing-borne doprinos:
`0 -> 9 -> 0 m/s`, `lambda: 1 -> 0.5 -> 1`, pusher do 0.35. Za razliku od L1,
live referenca koristi identificirani pitch/trim corridor i validan airspeed
stream. Nema PX4 transition komande niti gašenja lift motora.

Offline kandidat je prošao sa 8.991 m/s, `lambda=0.524`, max visinskom greškom
0.073 m, max pitchom 7.21° i bez solver failurea. Simulacija tačnog live
safety sloja također prolazi: 9.05 m/s, 0.026 m visinske greške, 7.73° pitch,
povratak na 0 m/s i `lambda=1`.

Offline provjera:

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_robust_allocation_l2_offline_gate.bash
```

Mora završiti sa `ROBUST_ALLOCATION_L2_OFFLINE=PASS`.

Terminal 1 — patched PX4/Gazebo, zatim PX4 konzola:

```text
param set VT_EXT_PUSH_EN 1
param set VT_EXT_PUSH_MAX 0.35
param set VT_EXT_PUSH_SLEW 0.10
param set VT_EXT_ALLOC_EN 1
param set VT_EXT_AL_SLEW 0.10
param show VT_EXT_PUSH_EN
param show VT_EXT_PUSH_MAX
param show VT_EXT_PUSH_SLEW
param show VT_EXT_ALLOC_EN
param show VT_EXT_AL_SLEW
```

Terminal 2 ostaje DDS agent iz L1 postupka. Svi ROS terminali moraju nakon
`source scripts/source_ros2_nmpc.bash` ispisati
`domain=0,discovery=LOCALHOST` u `$PX4_MPC_ROS_ENV`.

Terminal 3:

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
ros2 launch px4_mpc standard_vtol_robust_l2_gate_launch.py
```

Mora pisati `guarded L2 allocation mode`. U QGC poletjeti u Position modu na
12–15 m, usmjeriti se prema slobodnom prostoru i potpuno smiriti letjelicu.

Terminal 4:

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_robust_allocation_l2_gate.bash
```

Upisati `YES`; ne komandovati VTOL transition. Test traje oko 49 s. PASS je:

```text
ROBUST_ALLOCATION_L2=PASS
```

Automatski FAIL nastaje za više od 1.0 m visinske greške, 0.8 m/s vertikalne
brzine, 2.0 m cross-tracka, 15° nagiba, stale/invalid airspeed, napuštanje MC
stanja, solver failure ili neaktivan allocation kanal. Konačni PASS dodatno
traži najmanje 8 m/s ground i calibrated airspeed, `lambda<=0.60`, stvarni
pusher i povratak ispod 0.5 m/s.

Prvi live L2 pokušaj 1. septembra 2026. prekinuo je ROS watchdog poslije
7.92 s sa `odometry_or_reference_stale`. ULog je pokazao da to nije bio pad
modela ili regulatora: PX4 odometrija je imala najveći razmak 24 ms, nije bilo
gubitka Offboard signala, maksimalna brzina bila je 2.97 m/s, visinski raspon
0.156 m i maksimalna vertikalna brzina 0.164 m/s. Uzrok je bio dvostruki test
starosti stanja, drugi nakon NMPC solvea od približno 15--25 ms.

Od narednog pokušaja stanje svježije od 0.20 s ide u solver, prekid od
0.20--0.45 s samo ponavlja posljednju već ograničenu sigurnu komandu i ne radi
novi solve, a neprekidan prekid od 0.45 s i dalje automatski traži Position sa
`odometry_stale_continuous`. Status prikazuje najveći uočeni razmak kao
`maxima=[...,state_gap=...]`. Ostale L2 sigurnosne granice nisu promijenjene.

Drugi live pokušaj istog dana stabilno je završio cijeli profil: 48.52 s
Offboarda, 9.117 m/s ground speed, 9.239 m/s CAS, 0.202 m maksimalne visinske
greške, 0.258 m cross-tracka i nula solver failurea. Formalno je bio FAIL samo
zato što je slobodni optimizer izabrao minimalni `lambda=0.646`, dok L2 dokaz
traži `lambda<=0.60`. PASS prag nije popušten. Ovaj rezultat je pokazao da
referenca sama nije dovoljan dokaz prenosa authorityja.

L2b zato postavlja vremenski promjenjive donje i gornje granice za `lambda`
direktno kao OCP input constraints. NMPC i dalje bira `lambda` unutar koridora;
to nije naknadna zamjena izlaza. Gornja granica je schedule `lambda + 0.05`, a
donja ostaje 0.50. Tačni live-layer offline test postiže `lambda=0.537`, 9.053
m/s, 0.026 m visinske greške, povratak na nultu brzinu i nema solver failurea.
Isti live L2 postupak ispod sada predstavlja L2b i mora proći prije L3.

L2b je zatim prošao live. Sačuvani PX4 ULog `2026-09-01/20_17_02.ulg`
potvrđuje 46.15 s Offboarda, 9.187 m/s ground speed, 9.219 m/s CAS, 0.319 m
ukupnog raspona visine, 0.205 m/s maksimalne vertikalne brzine, bez Offboard
signal loss ili invalid local position i uz MC stanje tokom cijelog intervala.

### R4-L3a — međukorak prema dubokom transferu

Direktni offline kandidat od 12 m/s i `lambda=0.2` nije pušten u let: exact
live-layer simulacija je pri kočenju dala QP infeasibility, više od 2 m
visinske greške i oko 20° pitcha. To je validan no-go rezultat, a ne razlog za
popuštanje limita. Stabilni L3a kandidat je zato `0 -> 10.5 -> 0 m/s`,
`lambda: 1 -> 0.35 -> 1`, ubrzanje 0.30 m/s² i pusher do 0.42. Offline daje
10.334 m/s, `lambda=0.350`, 0.377 m visinske greške, 0.130 m/s vertikalne
brzine, 8.29° pitcha, povratak na nultu brzinu i nula solver failurea.

Offline prerequisite:

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_robust_allocation_l3_offline_gate.bash
```

Za live L3a Terminal 3 pokreće
`standard_vtol_robust_l3_gate_launch.py`, a Terminal 4
`scripts/run_robust_allocation_l3_gate.bash`. PX4 parametri su pusher max 0.42
i isti allocation kanal kao u L2b. Tek L3a PASS otključava novi pokušaj
12 m/s / `lambda=0.2`; puna L4 tranzicija ostaje zaključana do tada.

Ako L3a start vrati `airspeed_stream_unavailable_for_l3`, negativan CAS u
mirnom hoveru sam po sebi nije kvar. Status sada ispisuje i `airspeed_age` te
`airspeed_source`. Zbog povremenih DDS razmaka preflight dopušta svježu poruku
do 0.75 s; `source=0` ili starost veća od 0.75 s i dalje blokiraju Offboard.

Prvi live L3a pokušaj prekinut je na 6.66 s zbog jedne
`allocation_channel_inactive` poruke. ULog je pokazao stabilan let (1.80 m/s,
0.058 m raspona visine, 0.139 m/s maksimalnog `|vz|`, bez Offboard loss-a).
PX4 označava kanal neaktivnim već nakon 200 ms bez allocation setpointa, a ROS
status je zabilježio DDS gap od 0.328 s. Node zato sada traži 0.35 s
kontinuirano neaktivnog kanala prije aborta i status ispisuje `inactive_for`.
Nevalidan setpoint i dalje abortira odmah; tokom kratkog zastoja PX4 sam vraća
`lambda` prema sigurnoj vrijednosti 1.0.

Kasniji L3a pokušaj od 2026-09-01 nije pao pri kočenju. ULog
`2026-09-01/20_41_04.ulg` pokazuje da je pri 10.79 m/s, na kraju ubrzanja,
visinska greška narasla na 1.20 m. NMPC je dostigao `lambda=0.4`, ali je
stvarni elevator bio samo oko 0.24 normalizovane komande dok je predikcijski
model sadržavao dodatni airspeed-scheduled trim. Lift-motor izlaz je istovremeno
pao sa oko 0.24 na 0.12. To je model/actuator-interface mismatch, a ne safety
limit koji treba olabaviti.

Zato je external-allocation poruka proširena sa bounded
`elevator_feedforward` kanalom. NMPC i model koriste isti trim, PX4 ga sabira
na FW pitch kanal prije control allocationa, a stale/invalid/izlazak iz
Offboarda odmah ga vraća na nulu. Hard limit je 0.25. PX4 status vraća requested
i applied vrijednost, a ROS gate bilježi `maxima.elevator_ff`.

Novi redoslijed je obavezan:

1. rebuildani PX4 i ROS workspace;
2. ponoviti L2b i zahtijevati `elevator_ff=0.250` uz postojeći PASS envelope;
3. tek nakon L2b PASS-a ponoviti L3a;
4. L4 ostaje zaključan dok L3a ne prođe bez visinske greške i solver failurea.

Ponovljeni L2b sa eksplicitnim elevator kanalom prošao je 2026-09-02:
48.56 s Offboarda, 9.606 m/s, 0.086 m maksimalne visinske greške, nula solver
failurea, `lambda_min=0.550` i PX4-potvrđen `elevator_ff=0.250`. ULog je
sačuvan kao
`validation_logs/accepted/robust_allocation_l2_elevator_ff_pass_2026-09-02.ulg`.
Time je zatvoren model/interfejs mismatch koji je srušio prethodni L3a.

Novi L3a live-layer offline gate zatim prolazi sa 10.370 m/s,
`lambda_min=0.350`, 0.330 m visinske greške, 0.107 m/s vertikalne brzine,
7.85° pitcha i nula solver failurea. L3a je ponovo otključan za jedan live
pokušaj.

Live L3a je prošao 2026-09-02: 66.00 s testne sekvence, 10.570 m/s,
10.585 m/s CAS, 0.356 m maksimalne visinske greške, 0.328 m cross-tracka,
`lambda_min=0.360`, pusher 0.307, PX4-potvrđen `elevator_ff=0.250` i nula
solver failurea. PX4 ULog sadrži 62.62 s stvarnog Offboard intervala i
sačuvan je kao
`validation_logs/accepted/robust_allocation_l3_elevator_ff_pass_2026-09-02.ulg`.

### R4-L3b — 11 m/s i dublji transfer

Direktan kandidat 12 m/s / `lambda=0.20` još ne prolazi exact live-layer
simulaciju: solver/pitch problem je uklonjen kontrolisanim povratom autoriteta,
ali ostaje 3.03 m visinske greške. Safety limit nije proširen. Naredni
flight kandidat je zato 11 m/s, ubrzanje 0.25 m/s², kočenje 0.40 m/s²,
`lambda_min=0.30` i pusher do 0.42. Offline rezultat je PASS: 10.548 m/s,
1.019 m visinske greške, 0.179 m/s vertikalne brzine, 15.25° pitcha, uredan
povratak na nultu brzinu i `lambda=1`, bez solver failurea.

Prvo pokrenuti offline prerequisite:

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_robust_allocation_l3b_offline_gate.bash
```

Za live L3b Terminal 3 mora pokrenuti
`standard_vtol_robust_l3b_gate_launch.py`, a Terminal 4
`scripts/run_robust_allocation_l3b_gate.bash`. Terminal 4 provjerava cijeli
`l3_profile` i odbija stari L3a node. L4 (`lambda=0`) ostaje zaključan dok
L3b ULog ne potvrdi visinu, pitch, allocation/elevator kanale i povratak u MC
hover.

Prvi live L3b pokušaj je 2026-09-02 prekinut nakon 59.40 s sa
`l3_cross_track_limit`. Do prekida je ostvareno 11.181 m/s,
`lambda_min=0.314`, 0.761 m maksimalne visinske greške i aktivan
`elevator_ff=0.250`. ULog `2026-09-02/18_25_37.ulg` pokazuje da prvih približno
45 s cross-track ostaje ispod 0.7 m. Zatim nastaje S-zavoj: NED yaw odstupa
oko 14°, lateralna brzina raste do približno 2.5 m/s i tek potom cross-track
prelazi 2.5 m. Safety limit zato nije uzrok niti se povećava.

ULog je pokazao actuator/model mismatch. Pri `lambda≈0.31` NMPC traži yaw
rate do `+0.10 rad/s`, dok izmjerena yaw brzina ostaje približno
`-0.06 rad/s`. Standard VTOL ima dva elevona i elevator, ali nema rudder
control-surface yaw kanal. PX4 patch je istim `lambda` faktorom pogrešno
umanjivao vertikalni thrust i sva tri MC torque setpointa, dok CasADi model
zadržava upravljivu rate dinamiku. PX4 je zato izmijenjen tako da u L1-L3
`lambda` rasterećuje lift thrust i blenda MC/FW roll/pitch, ali MC yaw torque
ostaje raspoloživ jer ovaj airframe nema rudder. Prije ponovnog L3b leta obavezan je
potpuni PX4 rebuild/restart. Naknadni solver failurei nakon Position fallbacka
više se ne akumuliraju u završni status.

Ovo razdvaja dvije fizički različite odluke. Za L1-L3 vrijedi

```text
T_lift = lambda_lift * T_MC,
tau_MC,roll/pitch = lambda_lift * tau_MC,PID,
tau_MC,yaw = tau_MC,PID,yaw,
tau_FW,roll/pitch = (1-lambda_lift) * tau_FW,PID + elevator_ff.
```

Puna L4 tranzicija mora dodati torque allocation po osama. Posebno, direktni
yaw-rate zahtjev mora preći u koordinisani bank/course zakon prije nego
`mu_yaw -> 0`; tek tada `lambda_lift -> 0` dozvoljava gašenje lift motora.
L3b dokazuje lift transfer, ne još potpuni torque transfer.

Ponovljeni L3b sa axis-aware PX4 yaw patchom prošao je 2026-09-02:
81.55 s testne sekvence, 10.916 m/s forward speed, 10.918 m/s CAS,
0.304 m maksimalne visinske greške, 1.574 m maksimalnog ROS cross-tracka,
`lambda_min=0.324`, pusher 0.305, `elevator_ff=0.250`, nula solver failurea
i uredan povratak na `lambda=1` i Position mode. ULog potvrđuje 77.38 s
stvarnog Offboarda i yaw-rate RMS tracking grešku oko 0.0096 rad/s. Zbog
duge ukupne PX4 sesije sirovi ULog ima 275 MiB; prihvaćeni kompresovani dokaz
je `validation_logs/accepted/robust_allocation_l3b_yaw_authority_pass_2026-09-02.ulg.zst`
(93 MiB). Ovo potvrđuje da je prethodni L3b pad bio gubitak yaw authority,
a ne ograničenje lift/visinske dinamike.

L3b PASS ne otključava direktan skok na `lambda_lift=0`. Sljedeći razvojni
gate mora uvesti eksplicitnu per-axis torque težinu i koordinisani
bank/course zakon: prvo smanjiti direktni MC yaw autoritet pri približno
11 m/s bez dubljeg lift unloadinga, dokazati course hold, pa tek zatim ponovo
razmatrati 12 m/s / `lambda_lift=0.20`.

Promjena custom poruke zahtijeva gašenje PX4-a, Micro XRCE Agenta i svih ROS
nodeova pa pokretanje potpuno novih procesa. Poruka
`Change payload size ... 40 ... larger ... 35` znači da je u DDS grafu ostao
proces sa starom 35-byte definicijom; nije NMPC niti flight-dynamics kvar.
`source_ros2_nmpc.bash` sada provjerava nova polja i odbija stale overlay.

**Go/no-go prema punoj tranziciji:** jedan L2 live pokušaj se analizira prije
ponavljanja. Ako pokaže strukturirane pitch/altitude oscilacije ili ne može
zadržati envelope bez popuštanja navedenih limita, trenutni model/interfejs se
proglašava nedovoljnim za L3/L4 i prvo se radi reidentifikacija. Ako L2 prođe,
slijedi L3 (`11–13 m/s`, `lambda>=0.2`), a tek zatim L4 sa `lambda=0` i
gašenjem lift motora.
