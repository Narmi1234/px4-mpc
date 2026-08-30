# Standard VTOL NMPC — presjek rada za mentorski sastanak

Datum presjeka: **30. august 2026.**  
Repo: `/home/imran/Repositories/px4-mpc`  
Aktivni branch: `standard-vtol-identification`  
Posljednji stabilni commit prije tekućeg modeliranja: `23c8311`

## 1. Cilj istraživanja

Cilj nije da NMPC samo zatraži tranziciju koju zatim izvrši stock PX4 VTOL
kontroler. Cilj je da NMPC optimizira i komanduje samu hover-to-forward-flight
putanju, uključujući raspodjelu vertikalnog i aerodinamičkog autoriteta, dok
PX4 ostaje brzi i sigurni izvršni sloj.

Planirani NMPC izlaz je

```text
u = [c_lift, c_push, p_sp, q_sp, r_sp, lambda],
```

gdje su `c_lift` aggregate collective vertikalnih motora, `c_push` pusher,
`p_sp/q_sp/r_sp` reference ugaonih brzina, a `lambda in [0,1]` NMPC-ov
allocation weight (`1` = MC, `0` = FW).

NMPC neće optimizirati pojedinačne RPM-ove ni pojedinačne servo uglove. PX4
će zatvarati body-rate petlju i preko control allocatora realizirati pojedine
motore i površine. Time istraživački doprinos ostaje na robusnoj tranzicijskoj
kontroli, a ne na zamjeni svih PX4 low-level funkcija.

## 2. Predložena konačna podjela odgovornosti

| Funkcija | NMPC | PX4 |
|---|---:|---:|
| Putanja položaja, brzine, attitudea i airspeeda | optimizira | mjeri |
| Collective lift i pusher | komanduje | limitira i izvršava |
| MC/FW allocation weight `lambda` | optimizira | primjenjuje |
| Body-rate reference | komanduje | prati unutrašnjom petljom |
| Pojedinačni motori i servo izlazi | ne | control allocator |
| Estimator, airspeed, arm/disarm i Offboard | ne | autoritet |
| Watchdog i failsafe recovery | dodatni gateovi | konačni autoritet |
| Stock PX4 transition schedule | ne u NMPC modu | samo fallback |

Ovo je bitna razlika u odnosu na stari Gate D: u tom eksperimentu je PX4 i
dalje birao lift blend i trenutak završetka tranzicije. Gate D je zato koristan
negativan eksperiment, ali nije konačno PhD rješenje.

## 3. Repozitoriji i reproducibilni checkpointi

### `px4-mpc`

Branch `standard-vtol-identification`:

```text
7fc9452  docs: define full NMPC transition ownership
314bb20  analysis: identify VTOL closed-loop rate dynamics
23c8311  model: add torque-informed VTOL rotational replay
```

Raniji commitovi sadrže hover, pusher, Gate A/B/C i povučeni Gate D. Veliki
ULogovi i generisani CSV rezultati su namjerno izvan gita.

### `PX4-Autopilot`

Repo: `/home/imran/Repositories/PX4-Autopilot`  
Branch: `nmpc-external-pusher`  
Base PX4 revision: `5f1eae330b`

```text
0ea3b45221  feat(vtol): allow guarded offboard pusher input
7558a3d188  fix(vtol): update standard transition in offboard rates
```

PX4 branch je čist osim neversioniranog lokalnog `install/` direktorija.

## 4. Šta je promijenjeno u PX4-u

Commit `0ea3b45221` dodaje kontrolisani vanjski pusher u Standard VTOL:

- `VT_EXT_PUSH_EN`: eksplicitno enable, default siguran;
- `VT_EXT_PUSH_MAX`: hard limit pusher komande;
- `VT_EXT_PUSH_SLEW`: slew-rate limit;
- prihvatanje samo u armed Offboard body-rate režimu;
- stale/non-finite komanda vraća pusher prema nuli;
- stock put ostaje nepromijenjen kada override nije aktivan.

Commit `7558a3d188` omogućava da Standard VTOL transition state update radi i u
Offboard-rate režimu bez zahtjeva za zastarjelim MC/FW attitude setpointima i
bez prepisivanja vanjske body-rate putanje stock attitude referencom.

Izmijenjeni PX4 fajlovi su:

```text
src/modules/vtol_att_control/standard.cpp
src/modules/vtol_att_control/standard.h
src/modules/vtol_att_control/standard_params.yaml
src/modules/vtol_att_control/vtol_att_control_main.cpp
```

Važno ograničenje: trenutni PX4 patch **još nema eksplicitni NMPC-owned
`lambda` ulaz**. On dokazuje pusher put i Offboard-rate kompatibilnost. Novi
timestamped `lambda`/allocation interfejs, applied-value telemetrija i stale
recovery tek trebaju biti implementirani nakon offline validacije modela.

## 5. Gdje je model letjelice u kodu

Ne postoji samo jedan model; postoje tri nivoa, svaki s drugom svrhom.

### 5.1 Gazebo/SDF plant — izvor fizičkih parametara

Izvorni PX4 model:

```text
/home/imran/Repositories/PX4-Autopilot/
  Tools/simulation/gz/models/standard_vtol/model.sdf
```

NumPy preslikavanje:

```text
px4_mpc/px4_mpc/models/standard_vtol_gz_model.py
```

`StandardVtolGazeboModel` je 18-state validation plant:

```text
x_plant = [position(3), velocity(3), quaternion(4), omega(3), rotor_speed(5)]
u_plant = [motor_0..motor_4, servo_0..servo_2].
```

Koristi se samo offline da reprodukuje Gazebo sile i momente. RPM stanja nisu
planirana u online NMPC-u.

### 5.2 Stari reducirani model i postojeći OCP

```text
px4_mpc/px4_mpc/models/standard_vtol_gz_model.py
  StandardVtolTransitionRateModel
px4_mpc/px4_mpc/models/standard_vtol_casadi_model.py
px4_mpc/px4_mpc/controllers/standard_vtol_nmpc.py
```

To je 10-state model:

```text
x_old = [position(3), velocity(3), quaternion(4)]
u_old = [collective, pusher, p_sp, q_sp, r_sp].
```

Pretpostavlja da se body rate realizira trenutno. Taj model je bio dovoljan za
hover i MC forward gateove, ali nije dovoljan za tranzicijski pitch transient.
Postojeći CasADi/acados OCP i live ROS node još koriste ovu staru formulaciju;
zato se stari Gate D više ne leti.

### 5.3 Novi torque-informed tranzicijski model

```text
px4_mpc/px4_mpc/models/standard_vtol_rotational_model.py
  StandardVtolTorqueInformedModel
px4_mpc/px4_mpc/models/standard_vtol_rate_control.py
```

Novi offline model ima 16 state varijabli:

```text
x_new = [position(3), velocity(3), quaternion(4), omega(3), surface_state(3)]
u_new = [collective, pusher, p_sp, q_sp, r_sp, lambda].
```

To je **16 stanja, ne 16 DoF**. Letjelica ima šest fizičkih stepeni slobode;
kvaternion koristi četiri varijable za tri rotacijska stepena, a tri dodatna
stanja opisuju sporu dinamiku površina.

Novi model trenutno postoji u NumPy formi za replay/identifikaciju. Još nije
prenesen u CasADi/acados niti spojen na live node.

## 6. Jednačine novog modela

Interni NumPy validation model koristi Gazebo ENU/FLU konvenciju; ROS/PX4
interfejs koristi NED/FRD. Konverzije su eksplicitne u alatima.

```math
\dot p_W = v_W,
```

```math
\dot v_W = \frac{1}{m}R_{WB}(q)
\left(F_{motors}+F_{aero}\right)+g_W,
```

```math
\dot q_{WB}=\frac{1}{2}q_{WB}\otimes[0,\omega_B]^T,
```

```math
\dot\omega_B=J^{-1}\left(
\tau_{motors}+\tau_{aero}-\omega_B\times J\omega_B\right),
```

```math
\dot\delta_i=(k_i\delta_{cmd,i}-\delta_i)/\tau_i.
```

PX4 rate kontrola se reproducira kao

```text
omega_sp -> MC/FW PID + feed-forward -> normalized torque
         -> control allocation -> lift motors / servo commands.
```

Efektivni pitch moment trenutno koristi

```math
M_y=\bar q(c_0+c_\alpha\alpha+c_{\alpha2}\alpha|\alpha|
          +c_q q/V+c_{\delta_e}\delta_e)
    +c_m M_{motor,y}.
```

Zadnji član opisuje identificirano poništenje velikog nose-down rotor-drag
momenta i wing-lift momenta u transition blendu. Bez tog vezanog člana model
je imao veliku grešku i kada su mu dati budući stvarni actuator izlazi.

## 7. Porijeklo parametara

| Parametri | Izvor | Status |
|---|---|---|
| Masa, CoM, inercija | PX4 `standard_vtol/model.sdf` | deterministički izvedeno |
| Pozicije i ose 5 rotora | SDF link/plugin definicije | direktno |
| Motor constants, max speed, time constants, drag | SDF `MulticopterMotorModel` | direktno |
| Wing/elevator geometrija i LiftDrag koeficijenti | SDF `LiftDrag` | direktno |
| Servo limit ±45° | SDF + `SIM_GZ_SV_MINA/MAXA` iz ULoga | provjereno |
| MC/FW PID/FF gains | aktivni PX4 parametri iz ULoga/sourcea | reprodukovano |
| FW airspeed scaling/filter | PX4 FixedwingRateControl source + ULog | reprodukovano |
| MC motor allocation | least-squares rekonstrukcija na training ULogu | validation provjera |
| Surface allocation | PX4 servo output prema FW torque regresiji | `corr pitch≈0.999` |
| Surface lag | SDF joint damping/controller + ULog identifikacija | u aktivnom modelu `tau=1 s`, `gain=1` |
| Efektivni pitch moment | robustni fit na `run_01`, provjera na `run_02` | još nije finalno prihvaćen |

Numerički SDF-derived parametri aktivnog plant snapshot-a:

```text
mass = 5.02500003 kg
CoM_B = [-0.00021891, 0, 0.00027861] m
J_B diagonal ≈ [0.48043045, 0.34529256, 0.81685782] kg m²
hover command ≈ 0.5201195
hover rotor speed ≈ 784.98 rad/s
lift motor max = 1500 rad/s
pusher max = 3500 rad/s
```

SDF snapshot je vezan za PX4 revision `5f1eae330b`, tako da se model može
reproducirati i nakon budućih PX4 promjena.

## 8. Podaci i metod identifikacije

Glavni ULogovi:

| Log | Uloga | Posebnost |
|---|---|---|
| `standard_vtol_run_01.ulg` | training/development fit | stock trim 15 m/s |
| `standard_vtol_run_02.ulg` | development validation | drugi 15 m/s let |
| `standard_vtol_run_03_12ms.ulg` | dodatni razvoj | trim 12 m/s |
| `standard_vtol_run_04_18ms.ulg` | ekstrapolacijski development test | trim 18 m/s |
| Gate C | stock transition shadow | potpuni 3→1→4→2→3 ciklus |
| Gate D failovi | failure evidence | pitch/vertical transient |

Extractor poravnava podatke na 50 Hz i čuva:

```text
airspeed, VTOL state, body velocity, attitude,
rate setpoint, measured omega/omega_dot,
MC/FW virtual torque, integratore, gain compression,
motor/servo control i Gazebo bridge izlaze.
```

Alati:

```text
tools/extract_standard_vtol_rate_dataset.py
tools/validate_standard_vtol_rate_controller.py
tools/identify_standard_vtol_surface_dynamics.py
tools/identify_standard_vtol_pitch_moment.py
tools/validate_standard_vtol_rotational_replay.py
tools/validate_standard_vtol_rotational_rollout.py
tools/fit_standard_vtol_pitch_rollout.py
```

Model se fituje na jednom skupu, parametri se zamrzavaju, pa se mjeri 0.5 s
rollout na drugom skupu. Acceptance rollout ne smije koristiti buduće logged
torque/servo komande. Poseban `logged-actuator plant` ih koristi samo kao
dijagnostiku za razdvajanje greške plant-a od greške rate PID/allocatora.

Napomena o istraživačkoj validnosti: `run_02` je tokom razvoja već više puta
pregledan i više nije pošteno zvati ga potpuno netaknutim finalnim holdoutom.
`run_04` je također pregledan ovim presjekom radi 18 m/s ekstrapolacijske
dijagnostike. Zato se finalni model mora zamrznuti prije najmanje jednog novog,
unaprijed definisanog SITL holdout leta.

## 9. Eksperimentalni rezultati do sada

### Uspješni sigurnosni/flight gateovi

| Gate | Rezultat |
|---|---|
| Hover 10 s | PASS; max visinska promjena 0.194 m |
| Hover 30 s | PASS; max visinska promjena 0.186 m |
| External pusher 0→0.05→0 | PASS; stvarni motor 5 potvrđen |
| Gate A, MC 3 m/s | PASS; 26.576 s Offboard, max altitude error 0.200 m |
| Gate B1, MC 5 m/s | PASS; pusher 0.15, max altitude error 0.264 m |
| Gate B2, MC 8 m/s | PASS; pusher 0.179, max altitude error 0.241 m |
| Gate C, stock transition shadow | PASS kao data/model checkpoint |

Ovi testovi dokazuju stabilan Offboard-rate hover, aggregate lift, vanjski
pusher, speed feedback i siguran povratak do 8 m/s u MC stanju.

### Gate D — zašto je povučen

Gate D je više puta stigao do front transition/FW stanja, ali je padao na
`vertical_speed_limit`, `altitude_error`, `horizontal_speed_limit` ili solver
infeasibility. Tipični logovi pokazuju 10–14 m/s, veliki pitch transient,
gubitak visine i recovery u Position/MC.

Zaključak nije samo “treba još tuninga”. Stari 10-state OCP pretpostavlja
`omega=omega_sp`, dok PX4 bira stock lift blend. Time optimizer nema stanje ni
komandu potrebnu da predvidi i kontroliše preuzimanje pitch autoriteta. Zato je
Gate D zamrznut i služi kao motivacija za 16-state + NMPC-owned `lambda`.

### Identifikacija rate/rotacijske dinamike

1. Piecewise i LPV closed-loop rate modeli prolaze MC, ali blend/FW rollout
   ostaje nestabilan ili iznad praga.
2. PX4 rate PID reprodukcija je dobra: FW pitch virtual-torque RMSE približno
   `0.0052`, transition blend približno `0.0106` uz 1 s airspeed filter.
3. Motor allocator je vrlo precizan u MC-u (`≈0.001–0.002` command RMSE), ali
   stock blend rekonstrukcija je slabija (`≈0.06`).
4. Raniji efektivni model dao je 0.5 s pitch-rate RMSE:

```text
blend ≈ 0.114 rad/s
FW    ≈ 0.070 rad/s
acceptance threshold = 0.050 rad/s
```

5. Novi motor-drag/wing-lift coupled član daje:

```text
blend rate-sp rollout q RMSE       = 0.0854 rad/s
blend logged-actuator plant q RMSE = 0.0827 rad/s
FW rate-sp rollout q RMSE          = 0.0697 rad/s
```

Blend se značajno popravio, posebno plant-only rezultat (`≈0.469 → 0.083`),
ali formalni offline gate još nije prošao.

Direktni fit istih koeficijenata na 0.5 s endpoint grešku je smanjio training
`q` RMSE na `0.052 rad/s`, ali pogoršao `run_02` na `0.117 rad/s`. Kandidat je
zato eksplicitno odbijen kao overfit i nije upisan u aktivni model.

Na ponovo izvučenom 18 m/s `run_04` aktivni coupled model daje:

```text
blend q RMSE = 0.1003 rad/s   (ZOH 0.1133)
FW q RMSE    = 0.1210 rad/s   (ZOH 0.0780)
```

Model dakle poboljšava blend trend, ali ne generalizira FW pitch dinamiku na
18 m/s. To je dokaz za airspeed-scheduled residual, a ne za dalje podešavanje
jednog globalnog seta koeficijenata.

## 10. Trenutni status — šta radi, a šta još ne radi

Radi i dokumentovano je:

- SDF-derived 18-state validation plant;
- ULog extraction i frame konverzije;
- translacijska plant validacija na 12/15/18 m/s;
- hover/MC forward NMPC i vanjski pusher;
- PX4 rate PID i allocator replay;
- 16-state NumPy torque-informed model;
- automatizovani 0.5 s rollout gate;
- siguran koncept buduće podjele NMPC/PX4 odgovornosti.

Još ne radi:

- blend/FW pitch rollout nije ispod `0.05 rad/s`;
- 16-state model nije prenesen u CasADi/acados;
- `lambda` još nije eksplicitni OCP control u live solveru;
- PX4 nema timestamped external allocation-weight ulaz;
- nije izveden L1/L2/L3/L4 staged NMPC allocation let;
- puna tranzicija još nije izvedena sa NMPC kao vlasnikom blenda.

Zbog toga se trenutno **ne smije ponavljati** `scripts/run_transition_gate_d.bash`.

## 11. Predloženi naredni koraci

### Korak 1 — zatvoriti offline pitch model

1. Fitovati mali airspeed/`lambda`-scheduled pitch residual na `run_03` (12
   m/s) + `run_01` (15 m/s), bez budućih actuator komandi.
2. Razdvojiti stvarni pitch plant residual od estimator/filter faznog pomaka.
3. `run_02` (15 m/s) i `run_04` (18 m/s) koristiti samo kao development
   provjeru znaka, stabilnosti i ekstrapolacije.
4. Zamrznuti strukturu, koeficijente i acceptance pragove.
5. Snimiti najmanje jedan novi unaprijed definisan holdout let.
6. Cilj: blend i FW `q` RMSE ≤ `0.05 rad/s`, pravilan znak i stabilan rollout.

Ako jedan globalni koeficijent ne prođe, sljedeći kandidat treba biti mali
LPV residual zavisan od airspeeda i `lambda`, uz constraint na pozitivno
prigušenje. Ne dodavati individualne RPM stateove niti proizvoljan neuralni
model prije ovog testa.

### Korak 2 — prenijeti 16-state model u CasADi/acados

- dodati `omega(3)` i `surface_state(3)`;
- dodati `lambda` kao šestu kontrolu;
- ukloniti PX4-owned lift weight iz OCP parametara;
- dodati bounds/slew na `lambda`, collective, pusher, rate, alpha, pitch,
  altitude i vertical speed;
- generisati trim corridor sa eksplicitnim `lambda`;
- provjeriti solver p99 < 40 ms prije live rada na 20 Hz.

### Korak 3 — robustna offline zatvorena petlja

Simulirati MC→FW→MC za:

```text
nominalni plant,
±20% aero koeficijente,
masu/inerciju u definisanom intervalu,
headwind/crosswind scenarije,
airspeed bias i bounded model residual.
```

Acceptance: bez solver failurea, altitude error ≤2 m, vertical speed ≤1.5
m/s, |roll|/|pitch| ≤20°, monoton `lambda` u obje tranzicije.

### Korak 4 — PX4 external-allocation patch

Dodati novi eksplicitni i timestamped `lambda` kanal. PX4 primjenjuje tačno
jedan blend:

```text
T_lift = lambda * c_lift
tau_MC weight = lambda
tau_FW weight = 1-lambda
T_pusher = c_push.
```

Patch mora objavljivati applied `lambda`, odbiti stale/non-finite input i
kontrolisano vratiti `lambda→1`, pusher→0 i stock MC recovery.

### Korak 5 — staged SITL gateovi

| Gate | `lambda` profil | Cilj |
|---|---|---|
| L1 | `1→0.8→1` do 5 m/s | potvrditi external blend i recovery |
| L2 | `1→0.5→1` na 8–10 m/s | mixed rate authority |
| L3 | `1→0.2→1` na 11–13 m/s | skoro FW uz lift rezervu |
| L4 | `1→0→1` | puna NMPC front/back tranzicija |

Svaki gate ide redom: offline replay → shadow → jedan guarded SITL let → ULog
analiza → commit/checkpoint.

## 12. Predložene teme za razgovor s mentorom

1. Da li je istraživački doprinos dovoljno jasno postavljen kao **robustni
   transition NMPC sa optimiziranim allocation weightom**, a ne direct motor
   control?
2. Da li zadržati grey-box torque-informed model ili formalno preći na LPV
   residual/model-set formulation?
3. Koji robustni pristup koristiti prvo: scenario NMPC, tube/constraint
   tightening ili bounded disturbance estimator?
4. Koje finalne metrike porediti sa stock PX4: altitude loss, transition time,
   energy, peak pitch/vertical speed, wind robustness i constraint violations?
5. Koliko nezavisnih trim brzina, vjetrova i Monte-Carlo scenarija je dovoljno
   za doktorsku evaluaciju?

## 13. Glavni dokumenti i artefakti

```text
STANDARD_VTOL_ROBUST_NMPC_ARCHITECTURE.md   konačna ownership arhitektura
STANDARD_VTOL_RATE_IDENTIFICATION.md        identifikacija rotacijske dinamike
STANDARD_VTOL_REIDENTIFICATION.md           translacijska identifikacija
STANDARD_VTOL_PLANT_VALIDATION.md            SDF/Gazebo ULog postupak
validation_logs/*_SUMMARY.md                 prihvaćeni gateovi i fail evidence
results/standard_vtol_rate_identification/  generisani lokalni rezultati
```

Sirovi ULogovi ukupno zauzimaju stotine MiB i nisu u gitu. Summary fajlovi
sadrže putanju i SHA-256 kada je raw log ostavljen u PX4 SITL direktoriju.

## 14. Kratak zaključak

Rad je prešao iz tuninga starog 10-state Gate D kontrolera u opravdanu novu
arhitekturu. Pokazano je da Offboard-rate hover, pusher i MC ubrzanje do 8 m/s
rade, ali i da stock-PX4-owned blend ne može predstavljati konačni PhD
doprinos. Uspostavljen je reproducibilan ULog identifikacijski lanac i novi
16-state torque-informed model sa NMPC-owned `lambda` interfejsom.

Najbliži tehnički milestone nije novi let nego zatvaranje blend/FW pitch
rollouta, zatim CasADi port i PX4 `lambda` patch. Tek tada staged L1–L4 testovi
vode do prve pune NMPC-owned tranzicije.
