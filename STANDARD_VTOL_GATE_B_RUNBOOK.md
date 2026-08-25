# Gate B: MC pre-transition envelope

Ovo je jedini dokument za nastavak nakon prihvaćenog Gate A. Gate A je
zaključan u `validation_logs/PUSHER_FORWARD_GATE_A_SUMMARY.md`.

Gate B još uvijek **ne šalje VTOL transition komandu**. Cilj je odvojeno
dokazati da NMPC može sigurno doći do airspeed područja u kojem PX4 započinje
transition blend, dok vozilo ostaje MC.

## Redoslijed

```text
Gate A PASS, 3 m/s
  -> B0: implementacija i offline regresija, bez leta
  -> B1: jedan 5 m/s MC let, hover collective regulator ostaje aktivan
  -> B1 ULog: izmjeri stvarni wing-lift/collective rezidual
  -> B2: jedan 8 m/s MC let sa validiranim postepenim lift-unloadingom
  -> Gate C: PX4 transition shadow test
```

Ne preskakati direktno na `8 m/s` i ne pokretati stock trim iz
`trim_corridor.yaml` kao komandu. Taj trim od `5 m/s` naviše uključuje veliki
elevator equilibrium, dok je elevator u MC režimu zaključan; zato se prvo mora
izmjeriti stvarni MC collective rezidual.

## B0 — implementacija prije sljedećeg leta

Potrebno je implementirati i offline provjeriti:

1. Poseban `pretransition_5mps` test mode i servis; Gate A parametri ostaju
   nepromijenjeni.
2. Smooth speed profil `0 -> 5 -> 0 m/s`, početna akceleracija najviše
   `0.40 m/s^2`, hold `3 s` i najmanje `4 s` završnog hovera.
3. OCP i PX4 pusher hard limit `0.15`; ne koristiti generički OCP limit
   `0.60`.
4. Calibrated airspeed mora biti fresh, finite i valid prije starta i tokom
   aktivnog profila. Ground-forward speed ostaje feedback za path/geofence.
5. Dokazani Gate A cross-track regulator i simetrični pitch barrier ostaju
   aktivni.
6. Dokazani vertical-hover collective regulator ostaje stvarna komanda u B1.
   NMPC/trim collective se računa i loguje samo kao `shadow_collective`.
7. Status i analyzer moraju prijaviti ground speed, CAS, collective command,
   stvarne lift-motor izlaze, pusher, tilt, visinu, cross-track, VTOL state i
   solver statistiku.
8. Offline nominalni i disturbance test moraju proći prije live upute.

Dok B0 nije implementiran, nema korisničke live komande za Gate B.

## B1 — prvi 5 m/s MC let

Planirana fiksna konfiguracija:

```text
target ground-forward speed: 5.0 m/s
reference acceleration:      0.40 m/s^2
hold:                        3.0 s
pusher OCP/PX4 limit:        0.15
collective output:           dokazani vertical-hover regulator
NMPC collective:             shadow-only
VTOL state:                  MC cijelo vrijeme
```

Minimalni sigurnosni prostor: `100 m` ispred nosa, početna visina `15–20 m`.
Tačne terminalske komande se dodaju tek kada B0 kod, testovi, build i offline
gate prođu.

PASS kriteriji:

```text
4.25 <= peak forward speed <= 5.75 m/s
final horizontal speed <= 0.40 m/s
max altitude error <= 0.40 m
max vertical speed <= 1.0 m/s
max tilt <= 10 deg
max cross-track <= 1.0 m
actual pusher <= 0.155 i na kraju <= 0.005
fresh/valid airspeed tokom aktivnog profila
VTOL state = MC cijelo vrijeme
solver failures = 0
siguran povratak na hover
```

## Između B1 i B2

Iz B1 ULoga se računa koliko je collective regulator stvarno morao smanjiti
lift pri istoj visini i brzini. Tek taj izmjereni MC podatak postaje ograničeni
feedforward lift-unloading raspored. Raspored mora:

- početi od nule ispod `3 m/s`;
- biti kontinuiran i monotono rasterećivati lift motore;
- imati feedback korekciju visine iznad feedforwarda;
- vratiti puni hover collective pri kočenju i prije završnog hovera;
- biti ograničen tako da jedan model mismatch ne može ugasiti lift motore.

## B2 — 8 m/s MC pre-transition let

B2 se implementira tek nakon B1 ULog PASS-a. Početni pusher limit je `0.20`,
ali se može smanjiti ako B1 pokaže da nije potreban. Na `8 m/s` se prvi put
aktivira validirani lift-unloading feedforward uz altitude feedback.

PASS kriteriji ostaju najmanje jednako strogi:

```text
speed tracking error <= 0.75 m/s
final horizontal speed <= 0.50 m/s
max altitude error <= 0.40 m
max vertical speed <= 1.0 m/s
max tilt <= 12 deg
max cross-track <= 1.5 m
collective i pusher bez skokova
lift motori nikada ispod sigurnog MC minimuma
VTOL state = MC cijelo vrijeme
solver failures = 0
```

Tek B1 i B2 PASS otvaraju Gate C, gdje stock PX4 izvodi transition, a NMPC radi
shadow prediction bez preuzimanja transition aktuatora.
