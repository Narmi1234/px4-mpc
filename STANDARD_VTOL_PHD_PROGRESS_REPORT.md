# Standard VTOL NMPC — progress report za mentorski sastanak

Datum: **31. august 2026.**

Repo/branch: `px4-mpc / standard-vtol-identification`

Posljednji potvrđeni checkpoint: `3263d8a` (`R3b robust hover PASS`)

## 1. Sažetak koji treba iznijeti mentoru

Cilj je NMPC koji **sam optimizira hover-to-forward-flight tranziciju**.
NMPC treba određivati putanju, pusher, ukupni uzgon, body-rate reference i
raspodjelu MC/FW autoriteta. Stock PX4 transition scheduler ne smije odlučivati
kada se lift motori gase u konačnom rješenju.

```text
u = [c_lift, c_push, p_sp, q_sp, r_sp, lambda]
lambda=1: puni MC autoritet; lambda=0: puni FW autoritet.
```

NMPC ne komanduje pojedinačne RPM-ove ni pojedinačne servo izlaze. PX4
zadržava rate petlje, control allocation, estimator i failsafe.

Do sada je:

1. Gazebo/SDF plant preslikan u 18-state validation model;
2. iz ULogova identificirana rate i servo dinamika;
3. eksperimentalno odbačen nedovoljan 10-state tranzicijski model;
4. implementiran 16-state CasADi/acados NMPC sa `lambda` kontrolom;
5. offline front tranzicija prošla nominalni i četiri disturbance scenarija;
6. novi solver prošao read-only Gazebo shadow (`R3a`);
7. NMPC body-rate izlaz primijenjen u hoveru 5.05 s (`R3b`), bez solver
   failurea, uz poseban vertikalni safety loop;
8. započet eksplicitni PX4 `lambda`/allocation kanal.

Nije još urađeno:

- novi NMPC nije izveo tranziciju u letu;
- u R3b je bilo `pusher=0`, `lambda=1`;
- PX4 `lambda` patch je WIP, nije buildan, bench-testiran ni commitovan;
- back transition, vjetar i novi nezavisni holdout nisu završeni;
- optimizer je nominalni NMPC testiran na disturbance scenarijima, a ne još
  formalni min–max/tube robustni NMPC.

Korektna tvrdnja je:

> Implementiran je i djelimično u hoveru verificiran novi 16-state transition
> NMPC, ali NMPC-owned allocation i puna tranzicija još nisu eksperimentalno
> potvrđeni.

## 2. Šta radi NMPC, a šta PX4

| Funkcija | NMPC | PX4 |
|---|---:|---:|
| Prediction horizon i referentna putanja | vlasnik | ne |
| Aggregate collective `c_lift` | računa | limitira/alocira |
| Pusher `c_push` | računa | limitira/izvršava |
| Body rates `p_sp,q_sp,r_sp` | računa | prati rate PID-om |
| MC/FW weight `lambda` | računa | primjenjuje |
| Pojedinačni motori i površine | ne | control allocator |
| EKF, attitude, airspeed | ne | vlasnik |
| Arm, watchdog, failsafe | dodatni gate | konačni autoritet |
| Stock transition schedule | ne u NMPC modu | fallback/recovery |

Istraživački doprinos je tranzicijska putanja i koordinirani allocation, ne
zamjena svih low-level funkcija PX4-a.

## 3. Gdje je model u kodu

### 3.1 Fizički validation plant

Izvor parametara:

```text
/home/imran/Repositories/PX4-Autopilot/
  Tools/simulation/gz/models/standard_vtol/model.sdf
```

Implementacija:

```text
px4_mpc/px4_mpc/models/standard_vtol_gz_model.py
```

```text
x_plant = [position(3), velocity(3), quaternion(4), omega(3), rotor_speed(5)]
```

Ovaj 18-state model reprodukuje SDF sile, momente i rotore. Koristi se za
offline validaciju, ne kao online OCP.

### 3.2 Povučeni 10-state model

```text
x_old = [position(3), velocity(3), quaternion(4)]
u_old = [collective, pusher, p_sp, q_sp, r_sp]
```

```text
px4_mpc/px4_mpc/models/standard_vtol_casadi_model.py
px4_mpc/px4_mpc/controllers/standard_vtol_nmpc.py
```

Nema body-rate i surface stanja ni `lambda`. Prošao je hover/MC ubrzanje, ali
ne i tranzicijski pitch transient. Stari Gate D je zato povučen.

### 3.3 Aktivni 16-state NMPC model

```text
x = [p_W(3), v_W(3), q_WB(4), omega_B(3), delta(3)] in R^16
u = [c_lift, c_push, p_sp, q_sp, r_sp, lambda] in R^6
```

To je 16 state varijabli, **ne 16 fizičkih DoF**. Letjelica ima šest fizičkih
stepeni slobode. Kvaternion koristi četiri broja za tri rotacijska stepena, a
`delta` su tri spora stanja aerodinamičkih površina.

```text
px4_mpc/px4_mpc/models/standard_vtol_robust_casadi_model.py
px4_mpc/px4_mpc/controllers/standard_vtol_robust_nmpc.py
px4_mpc/px4_mpc/standard_vtol_robust_shadow_node.py
```

Prvi fajl je simbolički prediction model, drugi definiše OCP/acados, a treći
je ROS receding-horizon izvršna petlja.

## 4. Jednačine stvarnog online NMPC modela

Interno se koristi Gazebo ENU/FLU; PX4 je NED/FRD. Konverzije su u
`px4_mpc/px4_mpc/models/frames.py`.

### Translacija

```math
\dot p_W=v_W,
```

```math
v_{rel,B}=R_{WB}(q)^T(v_W-w_W),
```

```math
\dot v_W=\frac{1}{m}R_{WB}(q)
\left(F_{lift}(\lambda c_{lift})+F_{push}(c_{push})
+F_{aero}(v_{rel,B},\omega_B,\delta)+[b_x,0,b_z]^T\right)+g_W.
```

### Orijentacija

```math
\dot q_{WB}=\frac12q_{WB}\otimes[0,\omega_B]^T,
\qquad \|q_{WB}\|_2=1.
```

### Identificirana zatvorena body-rate dinamika

```math
\dot p=k_p(p_{sp}-p),\qquad k_p=6,
```

```math
\dot r=k_r(r_{sp}-r),\qquad k_r=4,
```

```math
\dot q_b=-a(V,\lambda)q_b+b(V,\lambda)q_{sp}
-4\mu(1-\mu)(c_0+c_\alpha\alpha-c_\theta\theta)-d_q,
\qquad \mu=1-\lambda.
```

`a(V,lambda)` i `b(V,lambda)` su bilinearna interpolacija četiri nenegativna
LPV čvora. `d_q` je bounded disturbance do približno `±0.47 rad/s²`.
Koeficijenti su u `standard_vtol_pitch_rate_model.py`.

### Površine

```math
\dot\delta_i=(\delta_{cmd,i}-\delta_i)/\tau_i,
\qquad \tau_i\approx1.0\;s,
```

```math
\delta_{cmd}=(1-\lambda)A_{surf}
\tau_{FW}(\omega_{sp}-\omega,V)+[0,0,\delta_{trim}(V)]^T.
```

Površine su stanja jer im je odziv reda `1 s`; motorne konstante su reda
`0.0125–0.025 s`, pa RPM nije online OCP stanje.

### Validation model naspram online modela

NumPy validation model eksplicitno koristi rigid-body jednačinu

```math
\dot\omega_B=J^{-1}(\tau_{motors}+\tau_{aero}
-\omega_B\times J\omega_B)
```

u `standard_vtol_rotational_model.py`. Online CasADi OCP koristi identificiranu
closed-loop rate dinamiku radi real-time izvođenja. Ne treba tvrditi da acados
trenutno integrira punu motor-torque jednačinu.

## 5. Gdje je NMPC optimizacioni problem

Svakih `50 ms` rješava se OCP sa `N=20` i horizontom `T=2 s`:

```math
\min_{x_{0:N},u_{0:N-1}}
\sum_{k=0}^{N-1}
(\|x_k-x_k^{ref}\|_Q^2+\|u_k-u_k^{ref}\|_R^2)
+\|x_N-x_N^{ref}\|_{Q_N}^2
```

uz

```math
x_0=\hat x(t),\qquad x_{k+1}=F_{RK}(x_k,u_k,w_k),
```

```math
0\le c_{lift},c_{push}\le0.70,\quad0\le\lambda\le1,
```

```math
|p_{sp}|,|q_{sp}|\le0.45,\quad|r_{sp}|\le0.30\;rad/s,
```

```math
|v_z|\le2\;m/s,\quad
-22^\circ\le\phi\le22^\circ,\quad
-22^\circ\le\theta\le18^\circ,
```

uz dodatne bounds na body rates i površine.

Solver je acados `SQP_RTI`, `PARTIAL_CONDENSING_HPIPM`, ERK i Gauss–Newton.
Računa cijelu predikciju, primjenjuje se prvi control sample, zatim se problem
ponovo rješava nakon nove odometrije.

## 6. Porijeklo parametara

| Grupa | Izvor |
|---|---|
| masa, CoM, inercija | SDF |
| rotor geometrija i motor constants | SDF pluginovi |
| aero geometrija/koeficijenti | SDF `LiftDrag` |
| PX4 MC/FW PID i FF | PX4 source + ULog parametri |
| MC allocation | ULog least-squares rekonstrukcija |
| surface allocation | FW virtual torque/servo regresija |
| surface lag | SDF joint + ULog identifikacija |
| LPV pitch model | 12 i 15 m/s ULogovi |
| disturbance bounds | rollout residual statistika |

```text
mass = 5.025 kg
J diagonal ≈ [0.48043, 0.34529, 0.81686] kg m²
hover command ≈ 0.52012
hover rotor speed ≈ 784.98 rad/s
lift motor max = 1500 rad/s
pusher max = 3500 rad/s
```

Model nije proizvoljno izabran: fizički dio dolazi iz SDF-a, unutrašnje petlje
iz PX4 sourcea/ULoga, a rezidual iz identifikacije. Novi unaprijed definisan
holdout ipak je još potreban za finalnu naučnu validaciju.

## 7. Eksperimentalni rezultati

Prethodno su prošli hover 10/30 s, external pusher, MC forward 3, 5 i 8 m/s.
Stock PX4 transition sa NMPC shadowom poslužio je za podatke.

Stari Gate D je više puta dostigao 10–14 m/s, zatim padao zbog pitch,
vertical-speed i altitude transienta. PX4 je i dalje birao stock blend. To je
negativan eksperiment koji motivira `omega`, surface states i NMPC-owned
`lambda`, a ne uspješna NMPC tranzicija.

Aktivni 16-state OCP offline prolazi:

```text
nominal front transition do 15 m/s, lambda≈0       PASS
poznati pitch disturbance ±0.235 rad/s²            PASS
nepoznati disturbance ±0.10 rad/s²                 PASS
konstantni ekstrem +0.47 rad/s²                    FAIL
```

Real-time gateovi:

```text
R3a ROBUST_HOVER_SHADOW=PASS
    publishes_fmu=False, solver_failures=0, p99=30.43 ms

R3b ROBUST_HOVER_OUTPUT=PASS
    Offboard=5.05 s, solver_failures=0, p99=26.97 ms
    automatski povratak u Position
```

R3b je prvi primijenjeni izlaz novog solvera, ali je namjerno bio ograničen:
bounded/slew-limited body-rate komande dolaze iz NMPC-a, collective se
zamjenjuje ranije validiranim vertikalnim safety loopom, pusher je nula, a
`lambda=1`. Zato je to partial live integration, a ne potvrda svih šest NMPC
izlaza niti tranzicija.

## 8. PX4 izmjene i trenutni WIP

Stabilni branch `nmpc-external-pusher` sadrži:

```text
0ea3b45221  guarded external Offboard pusher
7558a3d188  Offboard-rate transition compatibility
```

Poslije R3b implementiran je external-allocation patch:

```text
VtolNmpcAllocationSetpoint  # timestamp + lambda
VtolNmpcAllocationStatus    # requested/applied + active/valid
VT_EXT_ALLOC_EN             # default false
VT_EXT_AL_SLEW              # lambda slew
```

Planirana jedinstvena PX4 primjena je

```math
T_{lift}=\lambda c_{lift},\qquad
\tau_{MC}=\lambda\tau_{MC,PID},\qquad
\tau_{FW}=(1-\lambda)\tau_{FW,PID}.
```

Stale/invalid input vraća `lambda` prema jedan. ROS poruke i PX4 SITL su
uspješno buildani 2026-08-31. Guarded robust-hover node sada objavljuje
`lambda=1` i prati PX4 status `requested/applied/active/valid`; novi R3b PASS
zahtijeva da je PX4 kanal zaista bio active i valid. `lambda<1` još nije
flight-testiran ovim novim interfejsom.

## 9. Šta “robustan” sada znači

Implementirano je: bounded disturbance model, constraints, scenario matrica,
state/solver/Offboard safety gateovi.

Nisu implementirani: min–max NMPC, tube NMPC, chance constraints ni formalni
dokaz robustne stabilnosti/recursive feasibility. Sa mentorom treba izabrati
scenario/multi-model, tube/constraint-tightening ili disturbance-estimator
pravac.

## 10. Status i naredni put

| Sloj | Status |
|---|---|
| SDF/Gazebo plant | radi |
| identifikacija | radi; novi holdout nedostaje |
| 16-state CasADi/acados | radi |
| offline front-transition matrica | PASS |
| ROS shadow | PASS |
| live NMPC hover | PASS |
| PX4 `lambda` kanal | build PASS; live `lambda=1` re-test slijedi |
| L1 `lambda 1→0.8→1` | implementiran; offline PASS, live spreman |
| puna front/back tranzicija | nije izvedena |

Naredno:

1. ponoviti R3b i dokazati live `lambda=1` status handshake;
2. SITL bench dokazati `lambda=0.8` i stale recovery;
3. L1 offline → shadow → guarded live do 5 m/s;
4. analizirati ULog prije povećanja authority transfera;
5. L2 `lambda=0.5`, L3 `0.2`, L4 `0`;
6. puna front/back putanja, vjetar i Monte Carlo evaluacija.

## 11. Kratki odgovori za konsultacije

**Gdje su jednačine?** U poglavlju 4; izvršna simbolička verzija je u
`standard_vtol_robust_casadi_model.py`.

**Gdje je NMPC?** Cost, constraints i acados konfiguracija su u
`controllers/standard_vtol_robust_nmpc.py`; ROS receding-horizon petlja je u
`standard_vtol_robust_shadow_node.py`.

**Je li zaista NMPC?** Da: svakih 50 ms rješava nonlinear constrained OCP.
R3b je primijenio bounded body-rate dio prvog samplea uz zaseban collective
safety loop; to još nije live dokaz svih šest kontrola ni tranzicije.

**Zašto PX4 ostaje?** NMPC radi putanju/allocation; PX4 radi brzu rate petlju,
pojedinačne aktuatore, estimator i failsafe.

**Ko radi tranziciju?** U ciljnoj arhitekturi NMPC bira `lambda`, pusher,
collective i rate putanju. To je offline demonstrirano, ali live allocation
još nije potvrđen.

**Zašto 16 stateova?** Dodani su body rates i spore površine jer su izostajali
u neuspješnom 10-state modelu. RPM je dovoljno brz da ostane algebraički.

**Šta je doprinos?** Robusni transition NMPC sa kontinuirano optimiziranim
MC/FW allocationom, grey-box SDF/ULog modelom i PX4 sigurnim izvršnim slojem.

## 12. Odluke koje tražiti od mentora

1. Je li predložena ownership granica NMPC/PX4 prihvatljiva?
2. Koju formalnu robustnu formulaciju prioritizirati?
3. Je li identified closed-loop rate model dovoljan ili se traži puna
   rigid-body torque dinamika u online OCP-u?
4. Koje finalne metrike koristiti protiv stock PX4: altitude loss, vrijeme,
   energija, peak pitch/vertical speed, constraints i vjetar?
5. Koliko novih holdout letova i Monte Carlo scenarija je potrebno?

## 13. Glavni dokumenti

```text
STANDARD_VTOL_PHD_PROGRESS_REPORT.md        ovaj presjek
STANDARD_VTOL_PROFESSOR_DEMO.md             kratka demonstracija za sastanak
STANDARD_VTOL_ROBUST_NMPC_ARCHITECTURE.md   ownership i plan
STANDARD_VTOL_ROBUST_TRANSITION_RUNBOOK.md  operativni gateovi
STANDARD_VTOL_RATE_IDENTIFICATION.md        identifikacija
STANDARD_VTOL_PLANT_VALIDATION.md           SDF/Gazebo validacija
```
