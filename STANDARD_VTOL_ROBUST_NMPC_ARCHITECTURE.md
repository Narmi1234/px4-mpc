# Robustni NMPC za punu Standard VTOL tranziciju

Ovo je glavni dokument za nastavak rada. Cilj nije da NMPC samo zatraži
tranziciju koju zatim odradi stock PX4. Cilj je da **NMPC određuje samu
tranzicijsku putanju i raspodjelu uzgona**, dok PX4 ostaje sigurni real-time
izvršni sloj.

Stari Gate D sa PX4-owned lift blendom je završen kao koristan negativan
eksperiment. Ne ponavljati ga. Pokazao je da 10-state model i nepoznati PX4
blend nisu dovoljni za pouzdanu tranziciju.

## Šta će na kraju raditi NMPC

NMPC na 20 Hz optimizira cijeli hover-to-forward-flight manevar. ULog replay je
pokazao da se tri spora control-surface zgloba ne smiju tretirati kao trenutni,
pa je aktivno stanje

```text
x = [p_N, p_E, p_D, v_N, v_E, v_D,
     q_w, q_x, q_y, q_z, p, q, r,
     delta_left, delta_right, delta_elevator] in R^16.
```

To je **16 state varijabli, a ne 16 DoF**. Letjelica i dalje ima šest fizičkih
stepeni slobode. Kvaternion koristi četiri broja za tri rotaciona stepena i
mora zadovoljavati `q' q = 1`; zadnja tri stanja opisuju odziv servoa, a ne
nove stepene slobode.

NMPC izlaz je

```text
u = [c_lift, c_push, p_sp, q_sp, r_sp, lambda] in R^6,
```

gdje je:

- `c_lift` ukupna raspoloživa collective komanda četiri vertikalna motora;
- `c_push` komanda pusher motora;
- `[p_sp,q_sp,r_sp]` željene body-rate komande za PX4 unutrašnju petlju;
- `lambda` NMPC-ov lift/control-allocation weight, `1` u MC i `0` u čistom FW.

NMPC zato direktno odlučuje:

1. koliko brzo vozilo ubrzava;
2. koliko radi pusher;
3. željeni attitude i njegove body-rate komande;
4. kada i kojom brzinom se vertikalni motori rasterećuju;
5. kada aerodinamičke površine preuzimaju upravljanje;
6. kada je završena front tranzicija;
7. cijelu obrnutu putanju za back tranziciju;
8. ograničenja visine, vertikalne brzine, ugla napada, brzine, komandi i slew-a.

NMPC **ne upravlja pojedinačnim brzinama motora** i ne računa pojedinačne
otklone elevona. To nije potrebno za istraživački doprinos na nivou robusne
tranzicijske kontrole.

## Šta ostaje na PX4

| Funkcija | NMPC | PX4 |
|---|---:|---:|
| Putanja položaja, brzine, attitudea i airspeeda | vlasnik | mjeri/prenosi |
| Collective lift i pusher | računa | ograniči i izvrši |
| Raspored MC/FW autoriteta `lambda` | računa | primijeni u allocatoru |
| Body-rate reference | računa | prati brzom rate petljom |
| Pojedinačni lift motori | ne | control allocator |
| Elevoni/elevator | ne | FW rate loop + allocator |
| EKF, airspeed i attitude estimacija | ne | vlasnik |
| Arm/disarm i Offboard nadzor | traži | vlasnik i konačna odluka |
| Watchdog, actuator limiti i failsafe | postavlja dodatne gateove | konačni sigurnosni autoritet |
| Stock VTOL transition schedule | ne koristi se u NMPC modu | samo fallback/recovery |

Ključna granica je: PX4 smije realizirati NMPC komandu, ali u aktivnom NMPC
transition modu ne smije sam birati airspeed blend, trenutak gašenja lift
motora, pusher profil ili pitch tranzicijsku putanju.

## Komandni ugovor NMPC -> PX4

ROS/PX4 putanja ostaje `VehicleRatesSetpoint` za body rate i thrust, ali treba
novu eksplicitnu PX4 poruku ili uORB/ROS polje za `lambda`. Ne kodirati
`lambda` u neko nepovezano postojeće polje.

Na PX4 strani NMPC režim računa efektivne komande kao

```text
T_lift_effective = lambda * c_lift
MC torque weight = lambda
FW torque weight = 1 - lambda
T_pusher         = c_push
```

Rate izlazi se blendaju na nivou momenta/allocatora:

```text
tau_cmd = lambda * tau_MC_rate(p_sp,q_sp,r_sp)
        + (1-lambda) * tau_FW_rate(p_sp,q_sp,r_sp).
```

Ne smiju se istovremeno množiti `c_lift` u ROS nodeu i ponovo u PX4-u. Postoji
tačno jedan blend, u PX4 izvršnom sloju, a njegovu vrijednost zadaje NMPC.

PX4 patch mora:

- prihvatiti samo konačne i svježe komande u armed Offboard-rate modu;
- ograničiti `lambda` na `[0,1]`, pusher, collective i njihov slew;
- objaviti stvarno primijenjeni `lambda` i actuator izlaze radi validacije;
- ignorisati stock airspeed transition blend dok je external-NMPC mode validan;
- na stale komandu odmah zamrznuti/ukinuti pusher, vratiti `lambda -> 1` po
  sigurnom slew-u i pokrenuti stock MC recovery;
- nikada ne dopustiti da nestanak ROS-a ostavi lift motore ugašene.

## Model koji optimizer mora koristiti

Minimalne kontinuirane jednačine su

```math
\dot p_W = v_W,
```

```math
\dot v_W = \frac{1}{m}R_{WB}(q)
\left(F_{lift}(\lambda c_{lift})+F_{push}(c_{push})
+F_{aero}(v_B,\omega_B,\delta)+F_{drag}\right)+g_W,
```

```math
\dot q_{WB}=\frac{1}{2}q_{WB}\otimes[0,\omega_B]^T,
```

```math
\dot\omega_B=J^{-1}\left(\tau_{MC}+\tau_{FW}+\tau_{aero}
-\omega_B\times(J\omega_B)\right),
```

```math
\dot\delta_i=(k_i\delta_{cmd,i}-\delta_i)/\tau_i.
```

```text
tau_MC = lambda * allocator_MC(PID_MC(omega_sp-omega))
tau_FW = (1-lambda) * allocator_FW(PID_FW(omega_sp-omega), airspeed).
```

Ovo zamjenjuje netačnu pretpostavku `omega = omega_sp`. Prvi pokušaj
identifikacije čistog first-order closed-loop modela prošao je MC, ali nije
prošao blend/FW validaciju. Zato model eksplicitno zadržava rigid-body moment,
PX4 rate-controller saturaciju i SDF aerodinamički moment, bez dodavanja
pojedinačnih RPM stanja. Identificirani servo odziv je približno `1.03 s` za
oba elevona i `0.72 s` za elevator; zato su ta tri stanja zadržana u OCP-u.
Rezultati su u
[`STANDARD_VTOL_RATE_IDENTIFICATION.md`](STANDARD_VTOL_RATE_IDENTIFICATION.md).

Za robusnu formulaciju model dodatno koristi:

```text
w = [wind_N, wind_E, wind_D, aero_bias_x, aero_bias_z]
```

kao procijenjeni poremećaj/parametar. Prva implementacija može koristiti
bounded scenario NMPC ili constraint tightening; ne treba odmah praviti puni
min-max optimizer.

Pojedinačni rotor RPM ostaje samo u 18-state validation plantu, ne u online
OCP-u. Lift-motor time constant je `0.0125/0.025 s`, dok je identificirani
surface lag reda `1 s`; zbog te razlike rotor speed ostaje algebraički, a
surface ugao je stanje.

## Implementacijski put

### Faza 0 — zamrznuti stari Gate D

- Ne izvoditi više `scripts/run_transition_gate_d.bash`.
- Sačuvati prihvaćene Gate A/B/C logove i reprezentativne Gate D fail logove.
- Gate D koristiti kao dokaz zašto su potrebni `omega_B` i NMPC-owned
  `lambda`, ne kao tunerski problem.

Kriterij završetka: stari live script odbija pokretanje i dokumentacija vodi
na ovaj plan.

### Faza 1 — identificirati closed-loop rate dinamiku

Iz postojećih Gate C/D ULogova izvući na zajedničkom timestampu:

```text
airspeed, vtol_state, PX4 MC/FW weight,
rate setpoint [p_sp,q_sp,r_sp], measured [p,q,r],
actuator torque commands, attitude i elevator/elevon izlaze.
```

Fitovati first-order/MIMO model po zonama:

```text
MC:       V < 7 m/s, lambda blizu 1
blend:    7 <= V <= 12 m/s, 0 < lambda < 1
FW:       V > 12 m/s, lambda blizu 0
```

Validacija mora biti na odvojenom ULogu. Čisti first-order i LPV kandidati su
testirani i odbijeni: MC prolazi, ali blend/FW ne predviđaju pouzdano pitch
transient. Aktivni nastavak Faze 1 je torque-informed rigid-body model opisan u
rate-identification dokumentu. On se prihvata tek kada pravilno predviđa znak,
fazno kašnjenje i vrh pitch-ratea kroz FW ulazak.

### Faza 2 — novi 16-state model i OCP, samo offline

- [x] Dodati `omega_B` i tri surface-angle stanja u CasADi/acados model.
- [x] Dodati `lambda` kao šestu optimiziranu komandu.
- Ukloniti PX4 lift weight iz external parametara modela.
- Dodati constraintove na `lambda`, `Delta lambda`, angle of attack, altitude,
  vertical speed, attitude, rate, collective i pusher.
- Regenerisati trim corridor koji uključuje `lambda` i elevator trim.
- Replay svakog postojećeg Gate C/D loga raditi bez slanja komandi.

Prvi front-transition closed loop sada prolazi nominalno do 15 m/s i potpuno
unloaduje lift (`lambda≈0`). Prolaze i poznate blend perturbacije ±0.235
rad/s² te nepoznate ±0.10 rad/s². Back transition, vjetar, masa/inercija i
nezavisni Gazebo shadow još nisu završeni.

Minimalni offline acceptance:

```text
solver p99 < 40 ms                  # zatim optimizirati za 20 Hz deadline
solver failures = 0
altitude error <= 2 m
vertical speed <= 1.5 m/s
|roll|, |pitch| <= 20 deg
lambda monotono 1 -> 0 u front tranziciji
lambda monotono 0 -> 1 u back tranziciji
uspjeh za nominalno, +/-20% aero koeficijente i zadani vjetar
```

### Faza 3 — PX4 external-allocation patch, bez leta

Napraviti novi PX4 branch iz poznatog pusher checkpointa. Dodati:

1. external transition enable parametar, default `0`;
2. timestamped `lambda` input;
3. jedinstveni lift/torque blend;
4. applied-command telemetry;
5. stale/nonfinite watchdog i prisilni MC recovery;
6. SITL testove za `lambda=1`, `0.5`, `0` i prekid toka.

Bench/SITL provjera bez polijetanja mora dokazati mapiranje:

```text
lambda=1.0 -> lift i MC torque puni, FW torque nula
lambda=0.5 -> oba torque puta 0.5, lift 0.5*c_lift
lambda=0.0 -> lift/MC torque nula, FW torque pun
stale input -> kontrolisan povratak lambda na 1 i pusher na 0
```

### Faza 4 — staged allocation gateovi u letu

Svaki gate prvo offline, zatim shadow, pa jedan live SITL let:

| Gate | Komanda `lambda` | Pusher/airspeed | Šta dokazuje |
|---|---|---|---|
| L1 | `1 -> 0.8 -> 1` | do 5 m/s | external blend i povratak |
| L2 | `1 -> 0.5 -> 1` | 8–10 m/s | wing lift + mixed rate authority |
| L3 | `1 -> 0.2 -> 1` | 11–13 m/s | skoro FW bez gašenja sigurnosne rezerve |
| L4 | `1 -> 0 -> 1` | optimizirana putanja | puna NMPC front/back tranzicija |

Ni u jednom gateu PX4 stock transition scheduler ne određuje `lambda`.
Promjena između gateova je samo dozvoljeni envelope; ista NMPC formulacija
ostaje aktivna.

### Faza 5 — robustnost i istraživačka evaluacija

Nakon nominalnog L4 PASS-a izvesti kontrolisanu matricu:

- headwind, tailwind i crosswind;
- masa i položaj centra mase;
- degradirani aero koeficijenti;
- airspeed bias/dropout;
- ograničen pusher ili jedan slabiji lift motor samo u simulaciji;
- poređenje sa stock PX4 transition kontrolerom.

Metrike su altitude loss, transition time, energy, peak attitude/rate,
constraint violation, recovery success i solver vrijeme. To je jezgro PhD
doprinosa: NMPC zajednički optimizira ubrzanje, attitude, lift allocation i
robustne constraintove, umjesto PX4-ovog fiksnog airspeed/time blenda.

## Sljedeći konkretni posao

Ne pokreće se novi let. Redoslijed je:

1. [x] napisati ULog alat za rate-setpoint/measured-rate/torque dataset;
2. [x] generisati odvojeni train/validation report i odbiti neadekvatne
   first-order kandidate;
3. [x] implementirati torque-informed 16-state NumPy validation model sa PX4
   rate PID-om, allocatorom i identificiranim surface lagom;
4. [x] zamrznuti stabilni LPV pitch kandidat i prenijeti ga u CasADi/acados;
5. [x] proći nominalni i ograničeni disturbance front-transition offline gate;
6. [ ] validirati 16-state predikciju u Gazebo shadow režimu bez komandi;
7. [ ] završiti PX4 timestamped `lambda` interfejs i bench mapiranje;
8. [ ] pokrenuti L1, ne punu tranziciju.

## Definicija konačnog uspjeha

Rad je NMPC tranzicija samo ako ULog dokazuje da je kroz cijeli manevar:

- NMPC generisao `c_lift`, `c_push`, rate reference i `lambda`;
- PX4 primijenio isti svježi `lambda`, bez stock transition rasporeda;
- ulazak u FW i povratak u MC nastao iz NMPC putanje;
- sva ograničenja ostala zadovoljena i fallback provjeren;
- isti controller prošao nominalne i disturbance scenarije.

Ako PX4 sam bira lift weight ili transition schedule, to je shadow/hybrid
baseline, a ne konačni NMPC rezultat.
