# Put od potvrđenog pushera do NMPC tranzicije

Ovo je glavni dokument za nastavak rada. Ne pokretati proizvoljne kombinacije
starih skripti niti ručno povećavati pusher. Svaki naredni let ima jedan cilj,
fiksnu konfiguraciju i jasan PASS/FAIL kriterij.

## Krajnji cilj

Jedan NMPC na 20 Hz vodi Standard VTOL iz hovera u stabilan forward flight:

```text
hover
  -> kontrolisano ubrzanje u MC režimu
  -> PX4 front-transition state machine
  -> wing-borne forward flight
```

NMPC izlaz ostaje

```text
u = [collective_lift, pusher, roll_rate, pitch_rate, yaw_rate].
```

NMPC ne komanduje pojedinačne motore ni elevone. PX4 zadržava body-rate
regulatore, control allocator, VTOL state machine i failsafe zaštite.

## Trenutni dokazani checkpoint

Završeno je sljedeće:

- 30 s stabilan hover u Offboardu;
- MC ubrzanje, kočenje i zaustavljanje do `2.018 m/s`, bez pushera;
- custom PX4 branch `nmpc-external-pusher`;
- NMPC/PX4 pusher putanja do stvarnog motora 5;
- ograničeni `0 -> 0.05 -> 0` pusher test;
- ULog: peak `0.0500`, povratak `0.0000`, MC-only, maksimalna brzina
  `0.069 m/s`, altitude span `0.154 m`.

To znači da sada smijemo napraviti pusher zatvorenu petlju. Još ne smijemo
poslati komandu za VTOL tranziciju.

## Ko je vlasnik komande

| Faza | NMPC | PX4 |
|---|---|---|
| MC hover/ubrzanje (konačna arhitektura) | collective, pusher, body-rate reference | rate loop, allocator, lift motori i motor 5 |
| Front transition | collective, pusher, body-rate reference | VTOL state, MC/FW torque blend, lift-motor weight, allocator |
| FW flight | pusher i body-rate reference; collective reference je nula | FW rate loop, elevoni/elevator i allocator |
| Abort u MC | prekida setpointe i traži Position | stabilizira u Position modu |
| Abort u transition/FW | traži back-transition i nastavlja sigurni setpoint do MC | vraća lift motore, prelazi u MC; zatim Position |

Custom PX4 override trenutno upravlja pusherom u `MC_MODE` i
`TRANSITION_TO_FW`. U `FW_MODE` stock PX4 putanja uzima body-x thrust iz FW
rate setpointa. Prije pune tranzicije ULogom moramo potvrditi da nema skoka
pushera pri prelasku između te dvije putanje.

## Obavezne softverske izmjene prije sljedećeg leta

### 1. Jedinstven PX4/Gazebo timebase

Trenutni 12 s pusher gate dao je `12.00 s` u ROS wall vremenu i `10.436 s` u
PX4/Gazebo vremenu. To nije kvar pushera, ali moving reference za tranziciju
mora napredovati po vremenu kojim napreduje plant.

Node zato mora:

- koristiti PX4 timestamp iz svježeg odometry/status toka za profile i
  timeout;
- zadržati wall time samo za watchdog koji otkriva prestanak poruka;
- u statusu prikazivati `px4_elapsed`, `wall_elapsed` i procijenjeni
  `realtime_factor`.

PASS: offline test sa usporenim PX4 timestampom daje isti profil u funkciji
simulacijskog vremena, bez ranog završetka.

### 2. Airspeed kao stanje tranzicijskog gatea

Groundspeed je dovoljan za dosadašnji test bez vjetra, ali tranziciju određuje
airspeed. Node mora čitati `AirspeedValidated`, bilježiti fresh/valid status i
razlikovati:

```text
ground-forward speed = projekcija local velocity na smjer nosa
calibrated airspeed  = podatak koji PX4 koristi za VTOL blend/transition
```

Stale ili nevalidan airspeed blokira transition service. Nevalidan airspeed
tokom prvih MC gateova se prijavljuje, ali ne izaziva opasan trenutni abort.

### 3. Novi pusher-forward profil

Ne koristi se hard-coded pusher puls. Zadaje se glatka pozicija/brzina, a NMPC
računa pusher iz modela i feedbacka. Za prvi gate referentni attitude ostaje
blizu level flighta, tako da se ubrzanje ne može ostvariti samo velikim MC
naginjanjem.

Novi limiter mora biti odvojen od oba postojeća limitera:

```text
collective: 0.48 .. 0.56       # još uvijek hover envelope
pusher:     0.00 .. 0.10
pusher slew <= 0.03 /s
roll/pitch rate <= 0.20 rad/s
yaw rate <= 0.15 rad/s
tilt <= 10 deg
```

Pusher mora biti rezultat NMPC rješenja uz trim/feedforward referencu, a ne
vrijednost direktno kopirana iz vremenskog profila.

### 4. Telemetrija i automatska analiza

Status i ULog analyzer moraju za svaki gate dati:

- PX4 VTOL state i nav state;
- ground-forward speed i airspeed;
- komandovani i stvarni pusher;
- collective prije i poslije PX4 transition weighta kada je dostupan;
- altitude error, vertical speed, tilt, cross-track i displacement;
- solver status, solve-time p99 i razlog završetka;
- potvrdu da se pusher vratio na nulu.

## Gate A — PASS: 3 m/s pusher-feedback u MC režimu

Gate A je prihvaćen ULogom `2026-08-25/18_49_22.ulg`. Nema VTOL transition
komande i PX4 je cijelo vrijeme ostao `MC_MODE`. Trajni mali zapis rezultata je
u `validation_logs/PUSHER_FORWARD_GATE_A_SUMMARY.md`.

Konfiguracija:

```text
target ground-forward speed: 3.0 m/s
reference acceleration:      najviše 0.50 m/s^2
hold:                         2.0 s
pusher hard limit:            0.10
pusher NMPC/OCP limit:        0.10
pusher slew:                  0.03 /s
minimum final hover settle:   3.0 s
```

NMPC ubrzava pusherom i body-rateovima, drži brzinu, koči i zaustavlja se na
novoj hover tački. Uzdužna referentna pozicija je moving anchor na trenutnoj
poziciji, pa je ovo prvenstveno speed-tracking gate; cross-track, visina,
smjer i geofence ostaju apsolutno kontrolisani. Ova formulacija zamjenjuje
apsolutnu uzdužnu putanju koja je u live testovima stvarala sukob između
position i speed cilja i vidljivu ubrzaj-koči oscilaciju. Pitch envelope je
kontinuirana barijera, a ne hard prekidač. U ovom gateu provjereni
vertical-hover regulator još
zamjenjuje NMPC collective izlaz i čuva visinu. NMPC collective i stvarni
lift-unloading uključuju se tek u Gateu B, nakon što je pusher feedback zasebno
dokazan.

Prvi live test speed-reference formulacije potvrdio je longitudinalni dio
(`3.072 m/s`, altitude error `0.138 m`, tilt `5.31 deg`, solver failures `0`),
ali je ispravno abortirao na `cross_track=1.008 m`. ULog je otkrio rastuću
roll oscilaciju tokom kočenja, pa Gate A sada koristi zasebnu sporu,
prigušenu cross-track petlju umjesto raw NMPC roll izlaza. Offline provjera
prolazi i disturbance test sa trenutnim bočnim udarom `0.50 m/s`: dobijeno je
`3.10 m/s`, `4.85 deg`, `0.486 m` maksimalnog cross-tracka i nula solver
grešaka. Naredni live test potvrdio je cross-track (`0.529 m`) i sve ostale
gate kriterije, ali je otkrio da je pitch barrier bio jednostran: brake/settle
je dostigao `10.52 deg` na nose-up strani. Barrier je zato sada simetričan i
overspeed sloj samo smanjuje pusher, bez dodatne pitch komande. Sljedeći korak
je bio ponovni Gate A. Konačni let je zatim prošao sve kriterije:
`26.576 s`, `3.127 m/s`, final speed `0.053 m/s`, altitude error `0.200 m`,
tilt `3.22 deg`, cross-track `0.663 m`, stvarni pusher `0.0838 -> 0`, MC-only i
`ulog_gate=PASS`. Gate A se više ne ponavlja radi tuninga.

PASS zahtijeva sve:

```text
2.5 <= peak forward speed <= 3.5 m/s
final horizontal speed <= 0.35 m/s
actual pusher peak između 0.05 i 0.10
actual pusher na kraju <= 0.005
max altitude error <= 0.30 m
max vertical speed <= 0.75 m/s
max tilt <= 10 deg
max cross-track <= 1.0 m
VTOL state = MC tokom cijelog gatea
solver failures = 0
završetak = pusher_forward_test_timeout
```

Bilo koji drugi završetak je FAIL. Ne povećavati limit i ne ponavljati naslijepo;
prvo analizirati status i ULog.

## Gate B — MC pre-transition envelope

Tačan redoslijed implementacije i acceptance kriteriji su u
`STANDARD_VTOL_GATE_B_RUNBOOK.md`. B0 implementacija, nominalna simulacija i
disturbance simulacija su PASS. Neposredni naredni eksperiment je jedan B1
`5 m/s` MC let po tačnim komandama iz runbooka; još nema transition komande ni
aktivnog lift-unloadinga.

Tek nakon Gate A rade se odvojeni letovi na `5 m/s`, zatim `8 m/s`. Još nema
VTOL transition komande.

Gate B1 na `5 m/s` je prihvaćen ULogom `2026-08-26/05_52_44.ulg`: stvarni
pusher `0.150 -> 0`, peak speed `5.076 m/s`, final speed `0.034 m/s`, CAS
`5.359 m/s`, altitude error `0.264 m`, tilt `4.55 deg`, cross-track `0.177 m`,
MC-only i `ulog_gate=PASS`. Sažetak je u
`validation_logs/PRETRANSITION_GATE_B1_SUMMARY.md`. Neposredni naredni korak je
B2 implementacija/offline provjera, ne novi let.

Ovdje se dodaje:

- validirani airspeed feedback;
- level-flight MC trim schedule za collective i pusher;
- postepeno rasterećenje lift motora zbog mjerene wing lift sile;
- pusher limit `0.15`, a zatim samo ako treba `0.20`;
- test modela pri stvarnim pusher komandama.

Ne koristi se direktno wing-borne trim sa velikim elevatorom dok je
`VT_ELEV_MC_LOCK=1`. Pre-transition MC schedule i transition/FW trim corridor
su dvije različite grane.

PASS za svaki let:

```text
speed tracking error <= 0.75 m/s
altitude error <= 0.40 m
vertical speed <= 1.0 m/s
tilt <= 12 deg
cross-track <= 1.5 m
pusher i collective bez skokova
siguran povratak na hover i pusher=0
```

## Gate C — PX4 transition shadow test

NMPC radi samo shadow prediction dok normalni PX4/QGC napravi front i back
transition. `VT_EXT_PUSH_EN=0`, tako da PX4 u ovom testu potpuno upravlja
tranzicijom.

Gate B2 je prihvaćen ULogom `2026-08-26/06_28_37.ulg`, pa je Gate C sada
aktivni korak. Tačan postupak je u
[`STANDARD_VTOL_GATE_C_RUNBOOK.md`](STANDARD_VTOL_GATE_C_RUNBOOK.md).

Gate C stock front/back transition je zatim prihvaćen ULogom
`2026-08-26/17_14_25.ulg`. PX4 je ušao u FW na CAS `11.213 m/s`, prošao tačan
VTOL state slijed bez failsafea i vratio se u MC. Sažetak i model reziduali su
u `validation_logs/TRANSITION_SHADOW_GATE_C_SUMMARY.md`. Aktivni razvojni korak
je sada Gate D state machine; Gate C se ne ponavlja radi tuninga.

Ovim se provjerava:

- airspeed koji PX4 stvarno koristi;
- `MC -> TRANSITION_TO_FW -> FW` i obrnuti state slijed;
- default `VT_ARSP_BLEND=8 m/s`, `VT_ARSP_TRANS=10 m/s` i
  `VT_TRANS_MIN_TM=2 s`;
- stvarni PX4 lift-motor weight između 8 i 10 m/s;
- rate tracking i model reziduali kroz blend;
- back-transition i quad-chute događaji.

10-state prediction model prije Gate D mora uključiti isti efektivni lift
blend. Ako je `c_lift` raw NMPC komanda, fizički lift tokom transitiona je
približno funkcija `mc_weight * c_lift`; ne smijemo pretpostaviti da sva četiri
lift motora ostaju na punoj komandi.

## Gate D — prva NMPC front tranzicija

Početni uslovi:

```text
visina najmanje 30 m
mirni hover najmanje 10 s
najmanje 500 m čistog prostora ispred
airspeed fresh i valid
bez failsafea
```

Sekvenca:

1. NMPC ulazi u Offboard u MC hoveru.
2. NMPC ubrzava pusherom prema `8 m/s`.
3. Tek kada je airspeed stabilno iznad ulaznog praga i sve zaštite su zelene,
   node jednom šalje PX4 front-transition komandu.
4. PX4 ostaje vlasnik VTOL statea i MC/FW torque blenda.
5. NMPC prati `8 -> 12 m/s`, komanduje pusher i raw collective uz modelirani
   PX4 lift weight.
6. U `FW_MODE` PX4 lift weight postaje nula, pa raw collective više ne pokreće
   lift motore; NMPC pusher/rate kontrola nastavlja preko FW putanje.
7. Nakon kratkog holda node traži kontrolisani back-transition; test nije
   završen dok vozilo ponovo nije u MC Position modu.

Prvi transition pusher limit je `0.30`, jer corridor oko 10–12 m/s zahtijeva
približno `0.266`. Limit se ne postavlja dok Gateovi A–C ne prođu.

PASS:

```text
tačan VTOL state slijed bez quad-chutea
FW_MODE dostignut prije PX4 transition timeouta
airspeed pri ulasku u FW najmanje 10 m/s
max altitude loss <= 2.0 m
max roll/pitch <= 20 deg
pusher bez skoka većeg od 0.05 pri MC/transition/FW granicama
solver failures = 0
uspješan back-transition u MC
pusher=0 nakon povratka
```

## Gate E i F — proširenje envelopea

Nakon prve tranzicije povećavaju se samo reference, ne arhitektura:

| Gate | Profil | Cilj |
|---|---|---|
| E | `12 -> 15 -> 12 m/s` | stabilan wing-borne tracking i trim corridor |
| F | `12 -> 18 -> 15 m/s` | nominalni forward-flight envelope |

Svaki gate se prvo izvršava offline, zatim shadow, pa live. Tek poslije Gate F
uvodimo putanju/pozicijsko praćenje u FW i vjetar.

## State-dependent abort

Jedan `Position mode requested` nije dovoljan u svim fazama:

- u `MC_MODE`: pusher odmah na nulu i zahtjev za MC Position;
- u `TRANSITION_TO_FW`: otkazati front transition, pusher na sigurnu vrijednost
  i potvrditi povratak u MC prije prestanka setpoint toka;
- u `FW_MODE`: zatražiti back-transition, održavati siguran FW setpoint dok PX4
  ne vrati lift motore, pa tek u MC tražiti Position;
- stale odometry/airspeed, PX4 failsafe ili geofence uvijek imaju prioritet nad
  nastavkom reference.

Node mora imati eksplicitna stanja `idle`, `mc_accelerate`, `front_transition`,
`fw_hold`, `back_transition`, `mc_recovered` i `abort_recovery`. Ne praviti
tranziciju kao jedan timeout bez provjere PX4 potvrda.

## Šta se radi sada

Gate D software je implementiran i čeka prvi live let. State machine,
efektivni PX4 lift-weight u 10-state modelu, transition limiter, PX4 command
ACK, state-dependent MC recovery, launch argumenti, automatska skripta i ULog
`--gate-d` kriteriji su dodani. Jedini dozvoljeni postupak za prvi let je
[`STANDARD_VTOL_GATE_D_RUNBOOK.md`](STANDARD_VTOL_GATE_D_RUNBOOK.md).

Puni offline closed-loop Gate D PASS traje `58.95 s`: doseže `11.959 m/s`,
maksimalnu altitude grešku `1.385 m`, tilt `7.70 deg`, pusher `0.300`, nula
solver grešaka i završava u MC sa `0.074 m/s` i pusherom `0.0001`.

Gate D još nije PASS dok i ROS završetak i najnoviji ULog ne prođu. Gate E se
ne pokreće prije tog checkpointa.

Gate A software milestone je implementiran:

1. [x] PX4 timestamp timebase;
2. [x] `AirspeedValidated` subscriber i freshness status;
3. [x] 3 m/s level-attitude moving reference;
4. [x] NMPC pusher-feedback limiter do `0.10`;
5. [x] novi guarded service, launch parametri i automatski fallback;
6. [x] offline test i ULog analyzer;
7. [x] poseban runbook sa sva četiri terminala.

Offline closed-loop Gate A je označen PASS. Live verifikacija se izvodi samo po
[`STANDARD_VTOL_PUSHER_FORWARD_RUNBOOK.md`](STANDARD_VTOL_PUSHER_FORWARD_RUNBOOK.md);
tek u tom postupku se `VT_EXT_PUSH_MAX` privremeno postavlja na `0.10`.

Live verifikacija je potvrdila stvarni pusher `0.10` i MC-only stanje. Prvi
ULog je izdvojio bočnu oscilaciju, pa Gate A sada koristi zaseban roll limit
`0.10 rad/s`. Naredni testovi potvrdili su mirnu dinamiku, ali i lažne
`odometry_stale` abortove tokom DDS-offset handovera. Runtime timebase zato
uzima samo direktne raw PX4 timesync uzorke i između njih interpolira boot sat
monotonic wall satom i izmjerenim Gazebo real-time faktorom. DDS-prevedeni
apsolutni timestampovi više ne učestvuju u schedulingu ni freshness odluci.
ULog `06_16_38` zatim je pokazao stvarni dinamički overshoot: NMPC je interno
tražio pusher `0.60`, a izlaz i PX4 su ispravno izvršavali najviše `0.10`.
Gate A zato sada koristi zaseban OCP čija je pusher granica također `0.10`.
Nominalni i lateralno poremećeni offline profil prolaze s vrhom približno
`3.04 m/s`. Završni ponovljivi live ULog PASS je još otvoren prije Gatea B.

Posljednji live checkpoint je ULog `2026-08-25/06_31_37.ulg`. Raw PX4 sat i
oba freshness watchdoga radili su ispravno, OCP je potvrdio
`nmpc_pusher_max=0.100`, vozilo je ostalo MC, a visina, vertikalna brzina,
tilt i cross-track ostali su unutar granica. Gate je ipak prekinut u fazi
`accelerate` na `8.61 s`: ULog vrh je `3.550 m/s`, uz stvarni pusher vrh samo
`0.0776`. Timeline pokazuje pitch do približno `-6.25 deg`, pa ubrzanje ne
dolazi samo od pushera; live plant ima veći forward autoritet/manji efektivni
drag od reduced modela. Ne ponavljati let prije sljedećeg koraka: iz ovog ULoga
kvantificirati pusher/tilt doprinos i offline provjeriti measured-speed pusher
governor ili korekciju low-speed forward modela. Safety limit se ne povećava
samo da bi gate formalno prošao.

Ta analiza je završena preko devet Gate A ULogova: fitted tilt skala je
`0.986`, pusher skala približno `0.84`, a posljednji let pokazuje oko `75 ms`
body-rate kašnjenja. Glavni uzrok preleta je prebrza `0.75 m/s^2` referenca u
kombinaciji s tim unutrašnjim odzivom, ne jači pusher. Gate A sada koristi
`0.50 m/s^2`, timeout `26.5 s`, handover horizontalnu brzinu ispod `0.15 m/s`
i robustni measured-speed governor. Offline nominalni slučaj daje vrh
`3.019 m/s`; stres slučaj s `0.15 m/s` bočnim poremećajem, `1.35x` rate
efektivnošću i `0.15 m/s` forward udarom daje `3.071 m/s`, bez solver faila.
Sljedeći korak je jedan live Gate A po ažuriranom runbooku.

Live ULog `2026-08-25/17_09_48.ulg` potvrdio je da speed governor drži ROS
vrh ispod hard speed granice, ali je otkrio drugi realni limit: roll je ostao
`2.23 deg`, dok je pitch dosegao `10.32 deg`. NMPC je pri brzini ispod
reference zatražio maksimalno forward naginjanje, pa je test ispravno prekinut
sa `tilt_limit`. Gate A sada ima zaseban pitch control-barrier koji počinje na
`4 deg` i ne dopušta daljnju forward-pitch rate komandu. Offline stres sa
`3 deg` pitch udarom, `0.15 m/s` forward udarom, lateralnim poremećajem i
`1.35x` rate efektivnošću prolazi: vrh brzine `3.120 m/s`, tilt `4.83 deg`,
bez solver faila. Sljedeći korak ostaje jedan live Gate A; hard safety granice
nisu povećane.

Live ULog `2026-08-25/17_30_34.ulg` potvrdio je tilt PASS (`6.06 deg`), ali je
završio na `horizontal_tracking_error`: pitch barrier je pogrešno smanjivao i
pusher iako je brzina bila ispod reference. Pusher je zato dosegao samo
`0.0756`, a vrh brzine `2.279 m/s`. Ta sprega je uklonjena: attitude barrier
sada mijenja samo pitch-rate, dok pusher ostaje na NMPC zahtjevu; pusher se
smanjuje samo kada measured speed zaista pređe speed envelope. Nominalni i
poremećeni offline test i dalje prolaze, uključujući pitch udar.

Live ULog `2026-08-25/17_36_01.ulg` zatvorio je forward kanale: vrh brzine
`3.461 m/s`, tracking `1.660 m`, tilt `8.22 deg`, MC i svi freshness uslovi su
prošli. Abort je bio samo `altitude_error` na ROS vrijednosti `0.303 m`, tri
milimetra preko nepromijenjene granice `0.300 m`. Umjesto širenja safety
granice, vertical hover petlja je pojačana sa `kp=1, kd=2` na kritično
prigušene `kp=2, kd=2*sqrt(2)`. Offline nominalni maksimum altitude greške pao
je sa približno `0.216 m` na `0.123 m`; poremećeni slučaj daje `0.119 m` bez
solver faila.

Altitude handover sada zahtijeva svjež `VehicleLocalPosition.z_deriv` manji od
`0.10 m/s`. Isti signal zatvara vertikalni hover feedback jer je direktno
konzistentan s derivacijom položaja i Gazebo ground truthom; sigurnosni altitude
limit ostaje nepromijenjen na `0.30 m`.

Freshness zaštita razdvaja PX4 plant-age (`0.20 s`) od DDS wall transport-age
(`0.30 s`) i u statusu čuva maksimalne vrijednosti oba gapa. Ovo sprječava da
kratka pauza kompletnog usporenog simulatora izgleda kao gubitak samo odometry
izvora, a ostaje ispod PX4 Offboard-loss vremena.
