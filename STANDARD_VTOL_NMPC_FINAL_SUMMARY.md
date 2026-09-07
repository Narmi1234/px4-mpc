# Standard VTOL NMPC — završni tehnički presjek

Datum presjeka: 4. septembar 2026.  
Repo: `/home/imran/Repositories/px4-mpc`  
PX4 SITL branch: `nmpc-external-pusher`  
Posljednja potpuno prihvaćena demonstracija: **L4b**

## 1. Zaključak u jednoj stranici

Cilj rada bio je da NMPC, a ne ugrađeni PX4 transition controller, vodi
Standard VTOL iz hovera prema letu na krilu. To je zahtijevalo tri stvari:

1. model letjelice koji obuhvata hover, aerodinamiku i promjenu autoriteta;
2. NMPC koji optimizira kretanje i raspodjelu autoriteta;
3. PX4 interfejs koji izvršava NMPC komande, ali zadržava brze unutrašnje
   petlje, estimator, miksanje aktuatora i failsafe.

Implementirani sistem radi i dokumentovan je do **L4b**. U Gazebo SITL-u je
NMPC postigao 12.27 m/s, održao maksimalnu grešku visine 0.586 m, izvršio
lateralni manevar i smanjio direktni MC roll/pitch i yaw autoritet na približno
0.109. Solver je završio 108.05 s eksperimenta bez greške i sistem se automatski
vratio u Position mode. To je stvaran rezultat prenosa autoriteta, ali letjelica
je formalno ostala u MC stanju i lift motori nisu ugašeni.

U sljedećem L4c eksperimentu NMPC je zaista doveo lift-motor allocation na nulu
i lift motori su bili praktično ugašeni 7.30 s. Međutim, eksperiment je pao zbog
vertikalne/longitudinalne nestabilnosti. Zato je **L4c parcijalni dokaz motor-off
upravljanja, a ne uspješna tranzicija**. Trenutni identificirani model nije
dovoljno tačan u tom dijelu envelopea.

Najpoštenija ocjena je:

- postoji dovoljno materijala za kvalitetan progress report, poglavlja o
  modeliranju, integraciji, staged validation metodologiji i analizi negativnog
  rezultata;
- trenutno se ne može tvrditi da NMPC izvodi kompletnu VTOL tranziciju;
- trenutno se ne može tvrditi da je NMPC bolji ili robusniji od stock PX4
  kontrolera jer nije urađen uparen, statistički porediv eksperiment;
- nastavak ima smisla samo ako se prvo ponovo identificira wing-borne
  longitudinalni model iz L4c podataka i zatim zatvori puna nominalna
  tranzicija offline, prije novih live pokušaja.

## 2. Šta je urađeno

Razvoj nije bio jedno podešavanje kontrolera, nego niz provjerljivih koraka:

1. rekonstruisan je 6-DoF Gazebo plant iz PX4 Standard VTOL SDF-a;
2. snimljeni su PX4 ULogovi u više režima i brzinama, uključujući 12 i 15 m/s;
3. početni reducirani 10-state rate model je testiran i odbačen za tranziciju;
4. identificiran je LPV pitch-rate model i dopunjeni su roll, coordinated-turn
   i dinamika komandnih površina;
5. napravljen je 16-state CasADi/acados NMPC model;
6. PX4 je proširen svježim, ograničenim vanjskim pusher, elevator i per-axis
   allocation komandama;
7. uvedeni su shadow, hover, pusher i L1–L4 gateovi sa automatskim prekidom;
8. nominalni envelope je postepeno proširen do uspješnog L4b;
9. L4c je otkrio granicu modela pri stvarnom gašenju lift motora.

Detaljna hronologija je u
[`STANDARD_VTOL_ROBUST_TRANSITION_RUNBOOK.md`](STANDARD_VTOL_ROBUST_TRANSITION_RUNBOOK.md),
a ovaj dokument je autoritativni sažetak za mentorski razgovor.

## 3. Gdje je model i odakle dolaze parametri

### 3.1 Puni plant za validaciju

Datoteka
[`standard_vtol_gz_model.py`](px4_mpc/px4_mpc/models/standard_vtol_gz_model.py)
je programska rekonstrukcija PX4/Gazebo SDF modela. Geometrija, masa, inercija,
pozicije i smjerovi motora, motor constants, vremenske konstante i LiftDrag
koeficijenti dolaze iz `Tools/simulation/gz/models/standard_vtol/model.sdf`
na zabilježenoj PX4 reviziji `5f1eae330b`.

Puni plant ima 18 stanja:

$$
x_p=[p_W^T,\ v_W^T,\ q_{WB}^T,\ \omega_B^T,\ \Omega_1,\ldots,\Omega_5]^T,
$$

odnosno poziciju 3, brzinu 3, quaternion 4, ugaone brzine 3 i brzine pet
rotora 5. To je model letjelice sa šest fizičkih stepeni slobode; broj 18 je
broj stanja, ne broj DoF. Ulazi punog planta su četiri lift motora, pusher i
tri aerodinamičke površine.

Osnovne jednačine su:

$$
\begin{aligned}
\dot p_W &= v_W, \\
\dot v_W &= \frac{1}{m}R(q_{WB})
\left(F_{\mathrm{mot},B}+F_{\mathrm{aero},B}\right)
+\begin{bmatrix}0&0&-g\end{bmatrix}^{T}, \\
\dot q_{WB} &= \frac{1}{2}q_{WB}\otimes
\begin{bmatrix}0&\omega_B^T\end{bmatrix}^{T}, \\
\dot\omega_B &= J^{-1}\left(\tau_{\mathrm{mot},B}
+\tau_{\mathrm{aero},B}-\omega_B\times J\omega_B\right).
\end{aligned}
$$

Za svaki rotor vrijedi prvi red dinamike pogona i kvadratni thrust:

$$
\begin{aligned}
\dot\Omega_i &= \frac{\Omega_{i,\mathrm{cmd}}-\Omega_i}{\tau_i}, \\
T_i &= k_{T,i}\Omega_i^2.
\end{aligned}
$$

U implementaciji je masa približno 5.025 kg, bazna inercija je
`diag(0.47771, 0.34167, 0.81104) kg m²` prije composite korekcije, a analitička
hover komanda je približno 0.5201. To nisu proizvoljno pogođeni brojevi.

### 3.2 Zašto je 10-state model odbačen

Početni kontrolni model imao je

$$
\begin{aligned}
x_{10} &= \begin{bmatrix}p_W^T&v_W^T&q_{WB}^T\end{bmatrix}^{T}, \\
u &= \begin{bmatrix}c_l&c_p&p_{\mathrm{sp}}&q_{\mathrm{sp}}&r_{\mathrm{sp}}\end{bmatrix}^{T}.
\end{aligned}
$$

Pretpostavljao je da PX4 body-rate petlja trenutno ostvaruje
`omega = omega_sp`. Model je bio dovoljan za hover i blage MC gateove, ali nije
predviđao pitch-rate transient tokom tranzicije. Zbog toga je stari Gate D
povučen. Izraz „10-state“ ne znači „10 DoF“: letjelica i dalje ima 6 DoF, samo
je stanje reducirano.

### 3.3 Aktivni 16-state NMPC model

Aktivni model je u
[`standard_vtol_robust_casadi_model.py`](px4_mpc/px4_mpc/models/standard_vtol_robust_casadi_model.py):

$$
\begin{aligned}
x &= \begin{bmatrix}p_W^T&v_W^T&q_{WB}^T&\omega_B^T&\delta^T\end{bmatrix}^{T}
\in\mathbb{R}^{16}, \\
u &= \begin{bmatrix}c_l&c_p&p_{\mathrm{sp}}&q_{\mathrm{sp}}&r_{\mathrm{sp}}&\lambda\end{bmatrix}^{T}
\in\mathbb{R}^{6}.
\end{aligned}
$$

Ovdje su `c_l` kolektiv lift motora, `c_p` pusher, tri body-rate reference i
`lambda` traženi udio vertikalnog MC/lift autoriteta. `lambda=1` predstavlja
MC oslonac, a `lambda=0` planirano potpuno rasterećenje lift motora.

Model ima i osam online parametara:

$$
\theta=\begin{bmatrix}
w_x&w_y&w_z&b_{F_x}&b_{F_z}&d_q&\mu_{rp}&\mu_y
\end{bmatrix}^{T},
$$

gdje su vjetar, bias sile, bounded pitch disturbance i stvarno primijenjene
PX4 roll/pitch i yaw allocation težine.

Translacijska i quaternion dinamika ostaju oblika punog planta. Rotacijska
dinamika je grey-box model. Za pitch je identificiran LPV oblik

$$
\dot q=-a(V,\lambda)q+b(V,\lambda)q_{\mathrm{sp}}
-4\lambda(1-\lambda)
\left(c_0+c_\alpha\alpha-c_\theta\theta\right)-d_q,
$$

uz `|d_q| <= 0.47 rad/s²`. Koeficijenti se bilinearno interpoliraju između
četiri čvora u
[`standard_vtol_pitch_rate_model.py`](px4_mpc/px4_mpc/models/standard_vtol_pitch_rate_model.py).
Oni su fitovani iz 12 i 15 m/s ULogova, nisu uzeti iz SDF-a.

Roll model je

$$
\dot p=\mu_{rp}k_p(p_{\mathrm{sp}}-p)
-(1-\mu_{rp})a_p p+b_pV^2\delta_a,
$$

sa nominalnim `a_p=0.50` i `b_p=0.0475`. Identificirani pojedinačni fitovi su
obuhvatili približno `a_p=0.34...0.67` i `b_p=0.041...0.055`.

Koordinirani zaokret koristi

$$
\dot\psi_{\mathrm{coord}}=-0.95\frac{g\tan\phi}{\max(V,4)},
$$

a površine imaju prvi red

$$
\dot\delta=\frac{\delta_{\mathrm{cmd}}-\delta}{\tau_\delta},
\qquad \tau_\delta=1.0\,\mathrm{s}.
$$

Važno ograničenje: `lambda` je optimizirani NMPC ulaz, dok nezavisne PX4
roll/pitch i yaw allocation težine live čvor raspoređuje iz `lambda`. One još
nisu nezavisni optimizirani ulazi OCP-a.

### 3.4 Trag porijekla parametara

| Grupa parametara | Izvor | Status |
|---|---|---|
| masa, inercija, geometrija | PX4 Standard VTOL SDF | fizički nominal |
| motor constants i lag | PX4/Gazebo SDF | fizički nominal |
| LiftDrag koeficijenti | PX4/Gazebo SDF | simulator plant |
| hover collective | ravnoteža težine i četiri rotora | izračunato, ~0.5201 |
| pitch LPV čvorovi i disturbance | ULogovi 12/15 m/s | grey-box identifikacija |
| roll damping/surface gain | uspješni L3c i neuspješni L4a ULogovi | grey-box identifikacija |
| coordinated-turn gain | ULog replay | fit ~0.94–0.96, uzeto 0.95 |
| elevator trim schedule | force-balanced Gazebo trim corridor | model-based feedforward |
| težine cijene i guard pragovi | inženjersko podešavanje | nisu fizički identificirani |

Zadnja stavka se mora jasno odvojiti od identificiranih parametara. Cost
weights i sigurnosni pragovi nisu „parametri letjelice“.

## 4. Kako NMPC radi

OCP je implementiran u
[`standard_vtol_robust_nmpc.py`](px4_mpc/px4_mpc/controllers/standard_vtol_robust_nmpc.py).
Na svakom koraku rješava problem

$$
\begin{aligned}
\underset{x_k,u_k}{\operatorname{minimize}}\quad
&\sum_{k=0}^{N-1}\left(
\lVert x_k-x_k^r\rVert_Q^2+
\lVert u_k-u_k^r\rVert_R^2\right)
+\lVert x_N-x_N^r\rVert_{Q_N}^2, \\
\text{subject to}\quad
&x_{k+1}=f_d(x_k,u_k,\theta_k), \\
&x_k\in\mathcal X,\qquad u_k\in\mathcal U.
\end{aligned}
$$

Skupovi $\mathcal X$ i $\mathcal U$ predstavljaju granice aktuatora,
vertikalne brzine, attitudea, body rates i površina.
Horizont je 20 koraka / 2.0 s, čvor radi na 20 Hz, a solver je acados
SQP-RTI sa HPIPM QP solverom, ERK integracijom i Gauss–Newton Hessianom.

Prvi optimalni ulaz se izvrši, zatim se na novom stanju cijeli problem ponovo
riješi. Live čvor
[`standard_vtol_robust_shadow_node.py`](px4_mpc/px4_mpc/standard_vtol_robust_shadow_node.py)
dodaje reference profila, slew ograničenja, bumpless hover handover, provjeru
svježine podataka i solvera, CAS/allocation uslove, altitude/vz/tilt/cross-track
guardove i automatski povratak u Position mode.

Zbog toga je precizan naziv trenutnog sistema **nominalni constrained NMPC sa
robustness-oriented parametrima, guardovima i supervizijom**. To još nije
formalni robustni NMPC poput tube, min-max ili stochastic/chance-constrained
MPC-a.

## 5. Šta radi NMPC, a šta PX4

NMPC trenutno bira:

- ukupni lift collective;
- pusher komandu;
- roll, pitch i yaw body-rate reference;
- planirani lift/allocation faktor `lambda`;
- referentnu putanju i tempo prenosa autoriteta.

PX4 i dalje radi:

- EKF/state estimation i airspeed obradu;
- brze MC i FW rate petlje;
- control allocation i pojedinačne komande motora/serva;
- izvršenje vanjskog pusher/elevator/allocation zahtjeva;
- arming, Offboard watchdog, mode management i failsafe.

Ova podjela nije odustajanje od NMPC tranzicije. NMPC je outer-loop optimizer i
odlučuje o putanji i raspodjeli autoriteta, dok PX4 ostaje real-time inner-loop
autopilot. Problem starog Gate D bio je drugačiji: tada je PX4 sam birao ključni
transition blend. Custom PX4 branch je uveden upravo da tu odluku preda NMPC-u.

Custom PX4 branch sadrži, redom, podršku za vanjski pusher, bounded allocation,
elevator feedforward, odvojene per-axis težine i zadržavanje FW rate stanja.
Relevantna zadnja revizija je `b722b3e6bb` na branchu
`nmpc-external-pusher`.

## 6. Rezultati koji se smiju tvrditi

| Gate | Šta je dokazano | Ishod |
|---|---|---|
| hover shadow | stabilno online rješavanje bez izlaza | PASS |
| hover output | 5.05 s NMPC Offboard hover i siguran fallback | PASS |
| L1 | 5.31 m/s, `lambda_min=0.80` | PASS |
| L2 | 9.61 m/s, `lambda_min=0.55` | PASS |
| L3c | 12.20 m/s, `lambda_min=0.223`, povratak i kočenje | PASS |
| L4a | roll/pitch torque transfer do `mu_rp=0.092` | PASS |
| L4b | roll/pitch/yaw transfer, lateralni manevar, povratak | **PASS** |
| L4c | lift motori praktično ugašeni 7.30 s | parcijalno dokazano, **gate FAIL** |
| puna MC→FW→MC tranzicija | stabilan ulazak, FW hold i povratak | nije dokazano |

Prihvaćeni L4b rezultat:

- trajanje Offboard profila: 108.05 s;
- maksimalna ground/airspeed brzina: 12.260 / 12.274 m/s;
- maksimalna altitude/cross-track greška: 0.586 / 0.860 m;
- maksimalni pusher: 0.315;
- minimalni lift allocation: 0.250;
- minimalni MC roll/pitch i yaw autoritet: 0.109 / 0.109;
- solver status 0 i nula solver failures;
- uredan automatski povratak u Position mode.

Prihvaćeni kompresovani ULog je
[`robust_allocation_l4b_pass_2026-09-04.ulg.zst`](validation_logs/accepted/robust_allocation_l4b_pass_2026-09-04.ulg.zst),
SHA-256 `f8d539491f66cc4b312293b63830b61b040c2f3a76226d64f40df5b3526e5512`.
Sažetak je u
[`ROBUST_ALLOCATION_L4B_PASS_SUMMARY.md`](validation_logs/ROBUST_ALLOCATION_L4B_PASS_SUMMARY.md).

L4c negativni rezultat je također važan i reproducibilan: CAS je dosegao
13.654 m/s, allocation nulu, a srednji minimum lift izlaza bio je 0.0093 tokom
7.30 s. Ipak, `vertical_speed_limit` je prekinuo gate. Sažetak je u
[`ROBUST_ALLOCATION_L4C_MOTOR_OFF_LONGITUDINAL_FAIL_SUMMARY.md`](validation_logs/ROBUST_ALLOCATION_L4C_MOTOR_OFF_LONGITUDINAL_FAIL_SUMMARY.md).

## 7. Posljednja uspješna NMPC demonstracija

Za prezentaciju se koristi **L4b**, ne L4c i ne stari PX4-owned Gate D.
Stabilni ulaz je:

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_last_validated_nmpc_demo.bash
```

Skripta namjerno samo poziva zaključani L4b runner. Kompletan raspored četiri
terminala je:

### Terminal 1 — PX4/Gazebo

```bash
cd /home/imran/Repositories/PX4-Autopilot
git switch nmpc-external-pusher
make px4_sitl gz_standard_vtol
```

U PX4 `pxh>` konzoli:

```text
param set VT_EXT_PUSH_EN 1
param set VT_EXT_PUSH_MAX 0.45
param set VT_EXT_PUSH_SLEW 0.10
param set VT_EXT_ALLOC_EN 1
param set VT_EXT_AL_SLEW 0.05
```

### Terminal 2 — DDS agent

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
microxrce_agent_install/bin/MicroXRCEAgent udp4 -p 8888
```

### Terminal 3 — zaključani L4b NMPC node

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
ros2 launch px4_mpc standard_vtol_robust_l4b_gate_launch.py
```

### QGroundControl i Terminal 4

U QGC odabrati Position, armati i stabilizirati hover na 20–25 m. Ne pokretati
VTOL transition komandu. Zatim:

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_last_validated_nmpc_demo.bash
```

Očekivani završetak je `ROBUST_ALLOCATION_L4B=PASS`. Demonstracija pokazuje
kontinuirano ubrzanje do oko 12 m/s, dubok NMPC-owned transfer momentnog
autoriteta, mali lateralni out-and-back manevar, kočenje i povratak u Position.
Ne treba je nazvati punom tranzicijom ili FW letom.

## 8. Zašto se trenutno ne može tvrditi da je bolje od PX4-a

Stock/custom PX4 transition controller je u dosadašnjim probama kompletnu
tranziciju radio brže i pouzdanije. L4b i stock PX4 full transition nisu ista
misija, pa poređenje njihovih pojedinačnih brojeva ne bi bilo naučno korektno.

Za validno poređenje treba isti simulator commit, početno stanje, putanju,
vjetar, masu, senzorski šum i kriterije za oba kontrolera. Minimalne metrike su:

- stopa uspješnih tranzicija;
- vrijeme i pređena udaljenost tranzicije;
- maksimalna i RMS altitude greška;
- peak vertikalna brzina, pitch, body rates i actuator slew;
- približna energija/ukupna komanda aktuatora;
- broj povreda ograničenja i failsafe događaja;
- vrijeme NMPC računanja i deadline misses.

Tek nakon uspješne nominalne NMPC tranzicije treba pokrenuti, na primjer,
nominal, head/tail/crosswind i gust 2/4/6 m/s, masu ±10/20%, pomjeren CG,
aerodinamičke koeficijente, actuator degradation te delay/noise slučajeve.
Potrebno je više seedova po slučaju i intervali pouzdanosti, a ULogovi korišteni
za identifikaciju ne smiju biti test set.

## 9. Robusnost: šta postoji i šta nedostaje

Robusnost je trenutno samo djelimično obrađena kroz:

- bounded disturbance u pitch modelu;
- online parametre za vjetar i bias sile;
- constraint handling i slew/reachability ograničenja;
- stale-data, solver, altitude, vertical-speed, tilt i cross-track guardove;
- odvojene scenario/offline provjere i automatski fallback.

To dokazuje sigurnosno promišljenu implementaciju, ali ne dokazuje robustnu
stabilnost ili robustno zadovoljavanje ograničenja. Jedan raniji ekstremni
konstantni disturbance slučaj nije prošao, a puna nominalna tranzicija još ne
prolazi. Zato formulacija „robust NMPC“ u radu mora zasad značiti istraživački
cilj/arhitekturu, ne završen formalni rezultat.

Robusnost je moguće ispitati. Dva nivoa su:

1. **Empirijska robusnost:** Monte Carlo matrica navedenih perturbacija,
   zajednički baseline, holdout slučajevi i statistička analiza.
2. **Formalnija robusnost:** tube/min-max ili multi-model NMPC, constraint
   tightening, definisan skup neizvjesnosti i dokaz recursive feasibility ili
   barem invariant terminal seta.

Prvi nivo je realan sljedeći korak nakon nominalnog L4c uspjeha. Drugi nivo je
potencijalna jača PhD kontribucija, ali je značajno veći istraživački zadatak.

## 10. Problemi i ograničenja

Najvažnija tehnička ograničenja su:

- SDF plant je simulator ground truth, ali reducirani NMPC model nije tačan u
  motor-off wing-borne longitudinalnom području;
- identificirani pitch/roll modeli imaju ograničen train/validation envelope;
- elevator trim je model-based schedule, ne potpuno identificirana dinamika;
- cost weights, profili i guard pragovi su iterativno podešeni i mogu biti
  overfitovani na isti SITL;
- zasebni per-axis allocation faktori nisu svi optimizirani kao OCP ulazi;
- puni MC→FW→MC mode/state sequence nije uspješno zatvoren NMPC-om;
- nema hardware leta ni real-time platform validation;
- nema uparenog PX4 baseline eksperimenta ni statističke analize;
- nema formalnog dokaza robusnosti ili stabilnosti;
- česti DDS/schema i real-time-factor problemi otežavali su eksperimente, ali
  posljednji L4c kvar nije bio DDS ili solver kvar;
- regulator sadrži značajan supervisory sloj, pa u radu treba odvojeno opisati
  OCP izlaz, command shaping, PX4 inner loop i safety fallback.

L4c podaci konkretno pokazuju da problem nije samo „prenizak pusher“. Lift
motori su ugašeni i solver je ostao uredan, ali se javila longitudinalna
oscilacija. Dalje povećavanje brzine, limita ili popuštanje guardova bez novog
modela ne rješava uzrok i nije preporučeno.

## 11. Da li se od ovoga može napisati rad

**Da, može se napisati kvalitetan tehnički/progress rad**, sa sljedećim
provjerljivim doprinosima:

- traceable hibridno physics/ULog modeliranje Standard VTOL-a;
- eksplicitni NMPC/PX4 allocation interfejs;
- staged, abort-safe validacijska metodologija;
- demonstriran dubok transfer aktuatorskog autoriteta pri 12 m/s;
- motor-off eksperiment koji jasno lokalizira granicu trenutnog modela.

**Još se ne može odbraniti centralna tvrdnja** „predloženi robustni NMPC daje
bolju i robusniju punu VTOL tranziciju od PX4-a“. Za tu tvrdnju nedostaju puna
nominalna tranzicija, uparen baseline i kampanja robusnosti. Naučna novost u
odnosu na literaturu također se ne smije tvrditi bez zasebnog sistematskog
pregleda literature.

Realne opcije za razgovor s profesorom su:

1. nastaviti prema punoj tranziciji kroz novu identifikaciju L4c longitudinalne
   dinamike, pa tek onda baseline/robustness kampanju;
2. suziti temu na siguran NMPC-controlled authority transfer i sistematsku
   identifikaciju granice modela;
3. predstaviti trenutni rezultat kao metodološki i negativni rezultat, bez
   tvrdnje o superiornosti nad PX4-om.

## 12. Preporučena odluka i naredni rad

Za sada treba **zamrznuti L4b kao zadnju uspješnu demonstraciju i ne ponavljati
L4c live tuning**. Ako profesor potvrdi nastavak pune tranzicije, redoslijed je:

1. iz motor-off L4c intervala identificirati lift/drag/pitch i elevator
   dinamiku, uz odvojen train i novi holdout ULog;
2. validirati one-step i rollout grešku posebno za 10–14 m/s i
   `lambda=0...0.3`;
3. zatvoriti offline MC→motor-off→brake→MC simulaciju sa istim guardovima;
4. uvesti asimetričan allocation recovery: sporo rasterećenje, brzo ponovno
   uključivanje MC motora kada opadne airspeed ili poraste vertical/pitch error;
5. tek zatim dozvoliti jedan novi L4c SITL pokušaj;
6. nakon nominalnog PASS-a napraviti upareni PX4 baseline i empirijsku
   robustness matricu;
7. tek na osnovu tih rezultata odlučiti o formalnom robustnom NMPC proširenju.

Ovo je kontrolna tačka projekta: **dokazan je NMPC authority transfer do L4b;
puna NMPC VTOL tranzicija i superiornost nad PX4-om nisu dokazane.**
