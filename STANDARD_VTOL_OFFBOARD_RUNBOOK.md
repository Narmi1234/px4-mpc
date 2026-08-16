# Standard VTOL NMPC: staged hover-Offboard test

Ovaj test još **ne radi tranziciju**. Cilj je dokazati da NMPC može preuzeti
stabilan MC hover, držati trenutnu poziciju i sigurno vratiti PX4 u Position
mode. Node se nikad sam ne armira.

## Validirana regression procedura (30 sekundi)

Raniji pokušaj prekinut je nakon `6.36 s` zbog vertikalne oscilacije. Nakon
uvođenja prigušene hover-lift petlje, desetosekundni gate je prošao dva puta.
Bolji pokušaj imao je maksimalnu visinsku grešku `0.080 m`, horizontalni pomak
`0.128 m`, vertikalnu brzinu `0.048 m/s` i nagib `1.23 deg`. Ista konfiguracija
je zatim prošla 30-sekundni gate. Koraci ispod služe za ponavljanje tog
validiranog testa nakon budućih izmjena.

Za ovaj pokušaj uraditi tačno sljedeće:

1. Ako su stari procesi još pokrenuti, pritisnuti `Ctrl+C` u NMPC, DDS Agent i
   PX4 terminalu. QGroundControl može ostati otvoren.
2. Jednom izgraditi novu verziju:

   ```bash
   cd /home/imran/Repositories/px4-mpc
   source /opt/ros/jazzy/setup.bash
   source .venv/bin/activate
   colcon build --packages-select px4_mpc --symlink-install
   ```

3. Otvoriti Terminal 1 i pokrenuti PX4/Gazebo:

   ```bash
   cd /home/imran/Repositories/PX4-Autopilot
   make px4_sitl gz_standard_vtol
   ```

4. Otvoriti Terminal 2 i pokrenuti DDS Agent:

   ```bash
   cd /home/imran/Repositories/px4-mpc
   source scripts/setup_standard_vtol_nmpc.bash
   microxrce_agent_install/bin/MicroXRCEAgent udp4 -p 8888
   ```

5. Otvoriti Terminal 3 i pokrenuti novu verziju kontrolera. Ovaj terminal
   mora ostati otvoren:

   ```bash
   cd /home/imran/Repositories/px4-mpc
   source scripts/source_ros2_nmpc.bash
   ros2 launch px4_mpc standard_vtol_nmpc_launch.py \
     allow_offboard_output:=true \
     hover_offboard_max_seconds:=30.0
   ```

6. Otvoriti Terminal 4 i potvrditi samo da dolazi odometry. Komanda mora nakon
   nekoliko sekundi ispisati izmjerenu frekvenciju; zatim je prekinuti sa
   `Ctrl+C`:

   ```bash
   cd /home/imran/Repositories/px4-mpc
   source scripts/source_ros2_nmpc.bash
   ros2 topic hz /fmu/out/vehicle_odometry
   ```

7. U QGroundControlu ostaviti multicopter konfiguraciju, armirati, poletjeti u
   Position modeu na približno 5 m i potpuno pustiti komande. Sačekati najmanje
   5 sekundi da se letjelica umiri. Ne pokušavati handover dok se još penje ili
   spušta.
8. U Terminalu 4 pokrenuti jednu skriptu. Ona sama ispisuje početni status,
   traži Offboard, čeka cijeli 30-sekundni test i ispisuje konačni status:

   ```bash
   cd /home/imran/Repositories/px4-mpc
   bash scripts/run_next_hover_gate.bash
   ```

9. Držati QGC spreman. Ako se letjelica opasno nagne, brzo penje ili spušta,
   odmah ručno izabrati **Position mode**. Inače ništa ne dirati dok skripta ne
   završi.
10. Nakon tačno 30 sekundi kontroler treba sam vratiti Position mode. Na kraju
    skripte očekuje se `abort_reason=hover_test_timeout`. To nije greška: znači
    da je letjelica ostala u Offboardu do planiranog kraja. Svaki raniji razlog,
    npr. `altitude_error`, `vertical_speed_limit` ili `px4_left_offboard`, znači
    da test nije prošao. Terminal 3 sada automatski ispisuje i stvarno trajanje,
    grešku položaja/brzine i posljednju komandu, pa ne treba naknadno tražiti
    stare statuse.
11. Sletjeti kroz QGC, disarmirati i ugasiti PX4/Gazebo da ULog ne nastavi
    rasti. Novi test se prihvata tek nakon pregleda node loga i ULog intervala.

Ostatak dokumenta detaljno objašnjava iste komponente i služi za dijagnostiku.

## 0. Clean restart kada ROS/PX4 podaci nestanu

Ako `/status` pokazuje `nav_state=-1`, `state_age=infs` ili ROS ne vidi
`/fmu/out` topice, zaustaviti sve i krenuti ispočetka ovim redoslijedom:

1. u NMPC terminalu pritisnuti `Ctrl+C`;
2. u DDS Agent terminalu pritisnuti `Ctrl+C`;
3. u PX4/Gazebo terminalu pritisnuti `Ctrl+C`;
4. zatvoriti staru Gazebo instancu ako je ostala otvorena;
5. otvoriti četiri nova terminala i pratiti korake ispod.

Ne pokušavati Offboard dok DDS provjera iz koraka 5 ne prođe.

## 1. Jednokratni build

```bash
cd /home/imran/Repositories/px4-mpc
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --packages-select px4_mpc --symlink-install
```

Nakon uspješnog builda svi ROS terminali koriste samo jedan zajednički setup:

```bash
source /home/imran/Repositories/px4-mpc/scripts/source_ros2_nmpc.bash
```

## 2. Terminal 1: PX4 i Gazebo

Ne aktivirati `px4-mpc/.venv` u PX4 terminalu:

```bash
cd /home/imran/Repositories/PX4-Autopilot
make px4_sitl gz_standard_vtol
```

Sačekati da se Gazebo i PX4 potpuno podignu. QGroundControl može ostati
otvoren.

U PX4 konzoli provjeriti DDS client:

```text
uxrce_dds_client status
```

Ako nije `Running`:

```text
uxrce_dds_client stop
uxrce_dds_client start -t udp -h 127.0.0.1 -p 8888
uxrce_dds_client status
```

## 3. Terminal 2: DDS agent

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/setup_standard_vtol_nmpc.bash
microxrce_agent_install/bin/MicroXRCEAgent udp4 -p 8888
```

Ovaj terminal ne treba ROS 2 setup. Projektna skripta je potrebna zbog
`microxrce_agent_install/lib`. Ostaviti terminal otvoren i sačekati poruku da
se PX4 client povezao.

## 4. Terminal 3: NMPC s dopuštenim hover izlazom

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
ros2 launch px4_mpc standard_vtol_nmpc_launch.py \
  allow_offboard_output:=true \
  hover_offboard_max_seconds:=30.0
```

Prvi start može trajati oko 15 sekundi zbog generisanja acados C solvera.
Sljedeći startovi koriste već kompajlirani solver. Poruka mora završiti sa:

```text
Standard VTOL NMPC started in armed-capable guarded MC mode
```

Node još ništa ne šalje letjelici. Samo računa proposed control na:

```bash
ros2 topic echo /standard_vtol_nmpc/proposed_control
```

## 5. Terminal 4: obavezna DDS/ROS provjera prije armiranja

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash

which ros2
ros2 topic list | grep /fmu/out
ros2 topic hz /fmu/out/vehicle_status_v4
```

`which ros2` mora pokazati `/opt/ros/jazzy/bin/ros2`, a `vehicle_status_v4` mora
početi ispisivati frekvenciju. Prekinuti mjerenje sa `Ctrl+C`, zatim:

```bash
ros2 topic hz /fmu/out/vehicle_odometry
```

Kada i odometry pokazuje frekvenciju, prekinuti sa `Ctrl+C` i pozvati:

```bash
ros2 service call /standard_vtol_nmpc/status std_srvs/srv/Trigger '{}'
```

Dok je vozilo disarmed očekuje se `armed=False`, ali `nav_state` mora biti broj
različit od `-1`, `state_age` mora biti manji od `0.20s`, a `solve_time` mora
biti konačan broj. Ako se vidi `nav_state=-1`, `state_age=infs` ili
`solve_time=nanms`, komunikacija nije spremna: ne armirati i vratiti se na
clean restart iz koraka 0.

## 6. QGroundControl: armiranje i priprema

1. Letjelica mora biti u multicopter/hover konfiguraciji, ne u tranziciji.
2. Armirati i poletjeti normalno kroz QGC.
3. U Position modeu podići se na približno 5 m.
4. Pustiti da mirno lebdi najmanje pet sekundi.

Provjeriti node:

```bash
ros2 service call /standard_vtol_nmpc/status std_srvs/srv/Trigger '{}'
```

Traženo je `state_age < 0.20s`, `solver_failures=0`, `armed=True` i
`abort_reason=none`. `offboard=False` je u ovoj tački ispravno.

Ako node kaže `armed=False`, provjeriti stvarnu DDS vrijednost:

```bash
ros2 topic echo /fmu/out/vehicle_status_v4 --once --field arming_state
```

Vrijednost mora biti `2`. Ne zaobilaziti ovu sigurnosnu provjeru.

Prije Offboarda snimiti istu hover referencu u shadow-only režimu:

```bash
ros2 service call /standard_vtol_nmpc/capture_hover_reference \
  std_srvs/srv/Trigger '{}'
```

Tokom narednih 20 sekundi PX4 mora ostati u Position modeu. Nekoliko puta
provjeriti `/status`: pusher u `control=[lift,pusher,p,q,r]` mora biti `0.0`,
body-rate komande trebaju ostati male, a `solver_failures` mora ostati nula.
Ovaj servis nikada ne šalje setpoint PX4-u. Letjelica mora ostati armed i u
zraku tokom sva shadow mjerenja. Referenca čuva trenutni yaw, ali koristi level
roll i pitch kako NMPC ne bi pratio prolazni nagib PX4 Hold kontrolera. Disarm
automatski briše snimljenu referencu.

## 7. Preuzimanje hovera

Dok letjelica stabilno lebdi:

```bash
ros2 service call /standard_vtol_nmpc/enable_hover_offboard \
  std_srvs/srv/Trigger '{}'
```

Node snimi trenutnu poziciju, jednu sekundu šalje hover setpoint stream i tek
onda traži Offboard. Ne zadaje novu visinu, pusher je prisilno zaključan na
nulu, stvarno poslani body-rate je 50% NMPC prijedloga i dodatno ograničen na
0.20/0.20/0.15 rad/s. Watchdog prekida pri nagibu većem od 25 stepeni.
Regression test automatski vraća Position mode nakon trideset sekundi.

Za prvi hover gate stvarni lift izlaz je ograničen na `0.48–0.56` oko
ULog-izmjerenog hovera `0.5198`. Vertikalni sigurnosni sloj koristi modelski
izračunatu kritično prigušenu PD dinamiku sa prirodnom frekvencijom `1 rad/s`:

```text
a_z,cmd = -e_z - 2 v_z
lift = hover_lift + a_z,cmd / (d a_z / d lift)
```

Za validirani plant je `d a_z / d lift = 37.24 m/s^2`, a izlazni slew limit je
`0.10/s`. NMPC i dalje računa horizontalno kretanje i attitude/rate; ovaj
sporiji lift safety law ostaje aktivan i u narednim MC horizontalnim gateovima.

Handover se prihvata samo ispod 0.5 m/s horizontalne i 0.2 m/s vertikalne
brzine. Setpoint stream i prvih 0.5 s stvarnog Offboarda koriste
ULog-potvrđeni neutralni hover thrust `0.5201` i nulte rateove. Time se izbjegne
skok sa stare shadow komande. Watchdog prekida već na 0.5 m visinske greške ili
0.75 m/s vertikalne brzine. Strogi ulazni limit od 0.2 m/s provjerava se samo
pri handoveru; ne prekida normalnu malu korekciju nakon ulaska u Offboard.

Za ručnu dijagnostiku može se provjeriti:

```bash
ros2 service call /standard_vtol_nmpc/status std_srvs/srv/Trigger '{}'
```

Dok test traje, uspješan handover ima `offboard=True`, `solver_failures=0` i
`abort_reason=none`. `last_command_ack` za mode command `176` treba imati
`result=0`, što znači da je PX4 prihvatio naredbu. Za aktuelni test skripta iz
uvodnog postupka sama radi konačnu provjeru. Ako se Offboard ne uključi, ne
pozivati `enable` ponovo.

## 8. Normalan prekid i hitni izlaz

Normalan prekid:

```bash
ros2 service call /standard_vtol_nmpc/disable std_srvs/srv/Trigger '{}'
```

Node tada traži PX4 Position mode. U svakom trenutku pilot može ručno izabrati
Position mode u QGC-u; node detektuje napuštanje Offboarda i prestaje slati
upravljački izlaz.

Watchdog takođe prekida Offboard zbog zastarjelog odometryja, PX4 failsafea,
tri solver greške, izlaska iz MC konfiguracije, horizontalne brzine iznad
2 m/s, vertikalne brzine iznad 0.75 m/s, greške visine iznad 0.5 m ili
horizontalnog odmaka iznad 5 m.

## Acceptance gate

Tridesetosekundni hover test je prošao 2026-08-16. Izmjereno je `30.02 s` u
Offboardu, maksimalna visinska promjena `0.186 m`, horizontalni pomak `0.082 m`,
maksimalna vertikalna brzina `0.097 m/s` i maksimalni nagib `0.958 deg`, bez
failsafea ili solver greške. Detalji su u
`STANDARD_VTOL_HOVER_RESULTS.md`.

Gate kriteriji bili su:

- PX4 ostane u Offboardu bez failsafea;
- nema solver grešaka;
- visinska greška ostane ispod 0.5 m;
- automatski timeout ili QGC Position mode odmah vrate autoritet PX4-u.

Sljedeći milestone je ograničena horizontalna brzinska referenca u MC režimu,
prvo `2 m/s`, bez PX4 tranzicijske komande. Tek nakon tog gatea slijede
postepeni ciljevi 5, 10, 12 i 15 m/s i koordinisana PX4 VTOL transition komanda.

Prvi vremenski ograničeni gate od pet sekundi prošao je sa `0.16 m` konačne i
`0.468 m` maksimalne visinske promjene, bez solver grešaka. Nakon korekcije
horizontalnog weighta sljedeći pokušaj imao je samo `0.027 m` horizontalnog
pomaka, ali je prekinut nakon `6.36 s` zbog vertikalne oscilacije. Zbog toga je
uvedena nova prigušena hover-lift petlja, nakon čega su oba desetosekundna
gatea prošla. Verzijski kontrolisan sažetak svih hover gateova nalazi se u
`STANDARD_VTOL_HOVER_RESULTS.md`; numerički arhivi ostaju lokalno u ignorisanom
`validation_logs/` direktoriju.
