# Validacija Standard VTOL plant modela prema Gazebo simulaciji

Ovo je prvi korak prema hover-to-forward-flight NMPC-u. Cilj nije upravljati
motorima iz NMPC-a, nego provjeriti da matematički model daje približno iste
sile i momente kao `gz_standard_vtol` kada dobije iste ulaze.

Tokom ovog testa **PX4 upravlja letjelicom**. Ne pokreće se Offboard, MPC niti
stari identification/excitation node. Naš validator radi tek nakon slijetanja i
samo čita ULog; ne može poslati komandu letjelici.

## Koji model služi čemu

| Model | Gdje se koristi | Stanja motora | Ulazi |
|---|---|---:|---|
| `StandardVtolGazeboModel` (18 stanja) | offline poređenje s Gazebom | da, 5 brzina rotora | 5 ciljnih brzina motora + 3 površine |
| `StandardVtolTransitionRateModel` (10 stanja) | budući online NMPC | ne | collective lift, pusher i body-rate komande |

Brzine rotora su u punom modelu zato što Gazebo motor ne mijenja brzinu
trenutno. One su interne dinamike plant modela, a ne nešto čime se mora baviti
NMPC. Budući NMPC će optimizirati agregatne komande i oslanjati se na PX4 rate
kontroler i control allocation.

## 1. Jednokratna provjera okruženja

Lokalne putanje koje ovaj vodič koristi su:

```text
PX4:  /home/imran/Repositories/PX4-Autopilot
repo: /home/imran/Repositories/px4-mpc
QGC:  /home/imran/QGroundControl-x86_64.AppImage
```

Za validator nisu potrebni ROS 2 ni Micro XRCE-DDS Agent. Provjeri Python
pakete:

```bash
cd /home/imran/Repositories/px4-mpc
python3 -c "import numpy, pyulog, matplotlib; print('validation dependencies: OK')"
```

Ako neki paket nedostaje, može se instalirati u aktivni virtual environment:

```bash
python3 -m pip install -r requirements-validation.txt
```

Pokreni i brze testove modela i transformacija koordinatnih sistema:

```bash
cd /home/imran/Repositories/px4-mpc
PYTHONPATH=px4_mpc python3 -m unittest discover -s px4_mpc/test -p 'test_*.py'
```

## 2. Pokretanje QGroundControla i PX4 Gazebo SITL-a

Koristi dva terminala. Prvo pokreni QGC:

```bash
cd /home/imran
./QGroundControl-x86_64.AppImage
```

Zatim u drugom terminalu pokreni baš Standard VTOL model:

```bash
# PX4 build ne treba koristiti px4-mpc Python virtual environment.
deactivate 2>/dev/null || true
hash -r
command -v python3

cd /home/imran/Repositories/PX4-Autopilot
make px4_sitl gz_standard_vtol
```

`command -v python3` ovdje treba ispisati `/usr/bin/python3`, a ne
`/home/imran/Repositories/px4-mpc/.venv/bin/python`. Ako terminal nema
`deactivate` komandu, jednostavno otvori novi terminal i nemoj u njemu aktivirati
MPC `.venv`.

Ako CMake prijavi `No module named 'menuconfig'` ili `kconfiglib is not
installed`, build je pokupio pogrešan Python. U trenutnom lokalnom okruženju
sistemski Python već ima oba modula, pa ih nije potrebno ponovo instalirati;
deaktiviraj `.venv` i ponovi `make`.

QGC se treba automatski povezati na lokalni SITL. Prije armovanja provjeri:

- u QGC-u je prikazana jedna letjelica i status `Ready To Fly`;
- letjelica je u multicopter/hover stanju, a flight mode je `Position`;
- Gazebo koristi zadani svijet bez vjetra;
- nisu pokrenuti ROS Offboard node, MPC launch niti excitation node;
- QGC ne prikazuje failsafe ili preflight grešku.

PX4 po zadanim SITL postavkama snima ULog. Provjera loggera se unosi isključivo
u terminal u kojem je pokrenut PX4 i koji prikazuje PX4 prompt `pxh>`:

```text
pxh> logger status
```

Nemoj unositi `logger status` na obični Ubuntu prompt kao
`imran@imran:~$`; tamo je `logger` drugi, Linux system-log program. Aktiviranje
`px4-mpc/.venv` također ne otvara PX4 konzolu. Nemoj mijenjati model parametre
između snimanja i validacije.

### Veličina i čuvanje logova

Novi let pravi novi `.ulg`; prethodni se ne prepisuje. ULog je fajl na disku,
ne zauzima RAM nakon završetka procesa. Za kraće SITL logove postavi snimanje
samo od armovanja do disarma. U PX4 `pxh>` konzoli, prije leta, jednom unesi:

```text
pxh> param set SDLOG_MODE 0
```

Parametar zahtijeva restart PX4-a da bi nova vrijednost važila. Nemoj uključiti
`SDLOG_PROFILE` system-identification/high-rate bitove za ovaj test; validatoru
je dovoljan zadani profil. Najviše prostora se uštedi kratkim letom i gašenjem
SITL-a poslije disarma.

Validator može čitati originalni ULog direktno, pa kopija u `validation_logs/`
nije obavezna. Ako log ipak kopiraš, postoje dvije fizičke kopije dok jednu
ručno ne ukloniš. PX4 parametar `SDLOG_DIRS_MAX` ograničava broj datumskih
direktorija, a ne pouzdano broj letova napravljenih istog dana, zato nije zamjena
za povremenu provjeru sa `du -sh`.

## 3. Siguran i ponovljiv testni let

Prvi let neka bude kratak, približno dvije minute. Cijeli let se snima u jedan
ULog.

1. U QGC Fly prikazu izaberi `Position` mode.
2. Pritisni `Takeoff`, postavi visinu na **30 m** i potvrdi klizačem.
3. Nakon dostizanja 30 m miruj u hoveru 15 sekundi.
4. Komandom `Change Altitude` idi na 35 m, sačekaj 10 sekundi, pa se vrati na
   30 m i sačekaj još 10 sekundi. Ovo daje podatke za vertikalnu silu bez
   otvorene petlje.
5. Zadaj kratko horizontalno pomjeranje pomoću `Go To` na mapi, približno
   10–20 m, zatim miruj 10 sekundi. Time se pobude North/East ose.
6. Ponovo provjeri stabilan hover na najmanje 30 m. U gornjoj QGC traci klikni
   VTOL indikator i izaberi prelaz u `Fixed Wing`/forward flight. Naziv može
   biti `Transition to Fixed Wing`, zavisno od QGC verzije. Indikator je
   context-sensitive i može se pojaviti tek kada je vozilo armovano/u zraku;
   na novijem layoutu prvo proširi gornju traku tipkom `>`.
7. Pusti PX4 da završi tranziciju i leti 15–20 sekundi. Ne šalji motor ili
   thrust setpointe iz terminala.
8. U VTOL indikatoru zatraži `Transition to Multicopter`. Sačekaj stabilan
   hover 10 sekundi.
9. Pritisni `Land`, potvrdi i sačekaj potpuno slijetanje i automatski disarm.
10. Tek nakon disarma ugasi PX4 SITL. Time je ULog uredno zatvoren.

### Ako QGC ne prikazuje VTOL transition kontrolu

Za ovaj SITL možeš koristiti ugrađenu PX4 komandu. Unosi se u terminal koji
prikazuje `pxh>`, ne u obični Ubuntu terminal. Tek nakon stabilnog hovera na 30
m upiši:

```text
pxh> commander transition
```

Komanda je toggle: iz multicoptera traži forward-flight tranziciju. Nakon
15–20 sekundi leta ponovo je unesi da zatraži povratak u multicopter:

```text
pxh> commander transition
```

Stanje možeš provjeriti u istoj PX4 konzoli:

```text
pxh> listener vtol_vehicle_status
```

Ovo nije Offboard i ne šalje direktne motorne komande; šalje PX4-u standardni
VTOL transition zahtjev. Ako PX4 odbije zahtjev ili prijavi failsafe, nemoj ga
forsirati: ostani/vrati se u multicopter i sleti.

Ako se putanja udaljava više nego što želiš, odmah zatraži povratak u
multicopter preko VTOL indikatora, pa `RTL` ili `Land`. Ovo je SITL test, ali
visinska margina od 30 m je i dalje važna zbog tranzicije.

QGC reference za ove kontrole su [Fly View
Tools](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/fly_view/fly_tools.html)
i [Fly View
Toolbar](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view_toolbar.html).

Ako želiš prvo izolovati problem, uradi prvi let samo do koraka 5 pa sleti.
Validator će tada provjeriti hover model. Drugi let može sadržavati tranziciju.

## 4. Pronalaženje i čuvanje ULog-a

ULog se nalazi u PX4 SITL `rootfs/log` direktoriju. Prikaži najnovije fajlove:

```bash
find /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log \
  -type f -name '*.ulg' -printf '%T@ %p\n' | sort -nr | head
```

Kopiraj tačno odabrani log u lokalni, ignorisani direktorij ovog repozitorija.
U primjeru zamijeni datum i ime stvarnim rezultatom prethodne komande:

```bash
cd /home/imran/Repositories/px4-mpc
mkdir -p validation_logs
cp /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/YYYY-MM-DD/HH_MM_SS.ulg \
  validation_logs/standard_vtol_run_01.ulg
```

Nemoj commitovati velike `.ulg`, `samples.csv` ili slike; `validation_logs/` i
`results/` su namjerno u `.gitignore`.

## 5. Automatsko poređenje ULog-a s modelom

Pokreni validator iz korijena ovog repozitorija:

```bash
cd /home/imran/Repositories/px4-mpc
python3 tools/validate_standard_vtol_ulog.py \
  validation_logs/standard_vtol_run_01.ulg \
  --output results/standard_vtol_run_01
```

Za poređenje samo dijela leta, vrijeme se navodi u sekundama od početka loga:

```bash
python3 tools/validate_standard_vtol_ulog.py \
  validation_logs/standard_vtol_run_01.ulg \
  --start 40 --end 100 \
  --output results/standard_vtol_run_01_transition
```

Alat automatski:

1. čita stvarne motorne i servo izlaze koje je PX4 poslao Gazebu;
2. rekonstruiše kašnjenje pet rotora;
3. pretvara PX4 NED/FRD podatke u Gazebo ENU/FLU konvenciju;
4. preferira Gazebo ground-truth položaj, brzinu, akceleraciju i stav kada su
   prisutni u ULogu, umjesto da plant grešku pomiješa sa EKF dinamikom;
5. računa akceleracije iz punog 18-state plant modela;
6. poredi ih sa snimljenim linearnim i ugaonim akceleracijama;
7. odvaja multicopter, transition i fixed-wing faze.

Rezultat je:

```text
results/standard_vtol_run_01/
├── report.md
├── samples.csv
├── linear_acceleration.png
├── angular_acceleration.png
└── airspeed_and_motors.png
```

## 6. Kako odlučiti da li je model validan

Prvo otvori `report.md`, zatim dvije slike akceleracija. `prediction -
measurement` je definicija reziduala.

Provjera se radi ovim redoslijedom:

1. **Znakovi i frameovi:** pri usponu, ubrzanju naprijed i skretanju model mora
   promijeniti odgovarajuću komponentu u istom smjeru kao ULog. Obrnut znak je
   frame ili actuator-mapping greška, ne problem NMPC težina.
2. **Hover bias:** u mirnim hover prozorima cilj je apsolutni bias linearne
   akceleracije ispod `0.1 m/s²`. Prvo popravljamo masu, gravitaciju, trim ili
   mapiranje motora ako postoji konstantan pomak.
3. **Tranzicija:** gledaj da predikcija prati oblik i vrijeme promjene. Kratki
   vrhovi mogu biti kašnjenje/filter; dug pomak s rastom airspeeda upućuje na
   aerodinamički model.
4. **Forward flight:** početni cilj nakon pregleda ispravnih vremenskih prozora
   je NRMSE približno ispod 15% na pobuđenim osama. `n/a` znači da na toj osi
   nije bilo dovoljno mjerene akceleracije za smislen procenat.
5. **Faze:** report mora sadržavati `transition_to_fw` i `fixed_wing`. Ako ih
   nema, tranzicija nije snimljena ili nije završena.

Jedan dobar graf nije dovoljan. Gate za sljedeći korak je da barem tri ponovljena
leta daju iste znakove, sličan bias i sličnu grešku po fazama. Ne podešavamo
NMPC da prikrije sistematsku grešku plant modela.

## 7. Šta slijedi nakon ove validacije

Ovaj redoslijed je sada završen do hover-Offboard koraka:

1. generiše se trim corridor od hovera do približno 22 m/s;
2. 10-state transition model se implementira u CasADi/acados;
3. NMPC prvo radi u shadow modeu dok PX4 i dalje stvarno upravlja;
4. tek nakon provjere predikcije i vremena solvera uključuje se hover Offboard;
5. nakon uspješnog 30-sekundnog hover gatea slijedi MC horizontalni gate od
   2 m/s, pa koordinisana tranzicija kroz ciljeve 5, 10, 12, 15 i 18 m/s.

Kompletne jednačine i ostatak implementation patha su u
[`STANDARD_VTOL_TRANSITION_NMPC.md`](STANDARD_VTOL_TRANSITION_NMPC.md).
