# Ponovna identifikacija reduciranog Standard VTOL modela

Ovaj postupak identificira samo translacijsku aerodinamiku potrebnu prvom
10-state NMPC-u. Ne identificira pojedinačne motore niti 6-DoF momente. PX4
zatvara body-rate petlju, pa će NMPC zadavati collective lift, pusher i body
rates.

## Trenutni skup podataka

| Fajl | Uloga | FW trim | Upotreba |
|---|---|---:|---|
| `standard_vtol_run_01.ulg` | training | 15 m/s | procjena koeficijenata |
| `standard_vtol_run_02.ulg` | validation | 15 m/s | ne podešavati model prema njemu |
| `standard_vtol_run_03_12ms.ulg` | training | 12 m/s | dodatna airspeed/alpha pobuda |
| `standard_vtol_run_04_18ms.ulg` | validation | 18 m/s | ekstrapolacijski gate |

Oba leta imaju gotovo isti airspeed profil. Prvi kandidat poboljšao je body-x
acceleration RMSE sa `0.149` na `0.117 m/s²`, a body-z sa `2.411` na `1.533
m/s²` na netaknutom `run_02`. To još nije dovoljno za promociju u NMPC model:
potrebne su različite brzine kako bi se razdvojili angle-of-attack i elevator
uticaji.

`run_03` je završen sa ispravnim logging parametrima. Sadrži 31.5 s fixed-wing
faze sa prosječnim airspeedom 13.8 m/s. Privremeni fit na `run_01 + run_03`, uz
netaknuti `run_02` za validaciju, dao je body-x RMSE `0.085 m/s²` i body-z RMSE
`1.736 m/s²`. Horizontalni model se poboljšao, ali vertikalni još nije prošao
gate; `run_04` na 18 m/s je zato obavezan prije promjene NMPC modela.

`run_04` je završen sa 41.6 s fixed-wing faze i prosječnim airspeedom 17.2
m/s. Naknadnom provjerom ustanovljeno je da ULog već sadrži Gazebo
`vehicle_local_position_groundtruth` i `vehicle_attitude_groundtruth`. Validator
sada njih preferira umjesto EKF stanja za translacijsku plant validaciju.

Sa ground-truth podacima originalni SDF model na kombinovanom `run_02 + run_04`
validation skupu daje body-x/body-y/body-z RMSE `[0.097, 0.064, 0.431] m/s²`.
Na zasebnom 18 m/s letu body-z RMSE je `0.456 m/s²`. Time originalni fizički
model prolazi zadani `0.5 m/s²` FW gate. Ponovno identificirani polinom je
lošiji (`0.471 m/s²` kombinovano i `0.486 m/s²` na 18 m/s), pa je odbačen.

Zaključak: zadržava se SDF-derived plant bez empirijskog prepisivanja
koeficijenata. Novi letovi i dodatna Gazebo instrumentacija trenutno nisu
potrebni. Trim corridor 0–22 m/s, CasADi/acados OCP i hover-Offboard su nakon
ove odluke implementirani; 30-sekundni hover gate je prošao. Sljedeći
eksperiment je ograničena horizontalna brzina u MC režimu.

Trenutni kandidat i report su u:

```text
results/standard_vtol_parameter_fit_v1/
├── candidate_parameters.yaml
└── fit_report.md
```

## 1. Ponovno generisanje postojećeg kandidata

ULog prvo pretvori u prošireni validation CSV:

```bash
cd /home/imran/Repositories/px4-mpc

python3 tools/validate_standard_vtol_ulog.py \
  validation_logs/standard_vtol_run_01.ulg \
  --output results/standard_vtol_run_01

python3 tools/validate_standard_vtol_ulog.py \
  validation_logs/standard_vtol_run_02.ulg \
  --output results/standard_vtol_run_02
```

Zatim fituj samo na `run_01`, a rezultat ocijeni na `run_02`:

```bash
python3 tools/fit_standard_vtol_transition_model.py \
  --train results/standard_vtol_run_01/samples.csv \
  --validate results/standard_vtol_run_02/samples.csv \
  --output results/standard_vtol_parameter_fit_v1
```

Alat koristi samo validne fixed-wing uzorke iznad 10 m/s. Fitovani body-force
model je:

```text
alpha = -atan2(v_body_z, v_body_x)
beta  =  atan2(v_body_y, sqrt(v_body_x^2 + v_body_z^2))
qbar  = 0.5 * rho * V^2

Fx = qbar * (cx_0 + cx_alpha*alpha + cx_alpha2*alpha^2
             + cx_elevator*delta_elevator)
Fy = qbar * cy_beta*beta
Fz = qbar * (cz_0 + cz_alpha*alpha + cz_elevator*delta_elevator)
```

## 2. Dva potrebna dodatna leta

Potrebne su dvije različite brzine. Parametri mijenjaju PX4 referentnu brzinu,
ne Gazebo plant.

U `pxh>` konzoli prije **run_03** postavi:

```text
param set SDLOG_MODE 0
param set SDLOG_PROFILE 1
param set FW_AIRSPD_TRIM 12
```

Ugasi i ponovo pokreni PX4 SITL jer logging parametri zahtijevaju restart.
Nakon restarta provjeri:

```text
param show SDLOG_MODE
param show SDLOG_PROFILE
param show FW_AIRSPD_TRIM
```

Izvedi let:

1. takeoff i stabilan hover na 30 m;
2. 15 s hover;
3. `commander transition`;
4. nakon završene tranzicije 40–60 s stabilnog FW leta;
5. ponovo `commander transition` za MC;
6. 10 s hover, land i sačekaj disarm.

Za **run_04** ponovi isto, ali prije pokretanja leta postavi:

```text
param set FW_AIRSPD_TRIM 18
```

Za promjenu samo `FW_AIRSPD_TRIM` obično nije potreban restart, ali provjeri
vrijednost sa `param show`. Nakon snimanja vrati nominalnu vrijednost:

```text
param set FW_AIRSPD_TRIM 15
```

Ne mijenjaj `FW_AIRSPD_MIN=10` ni `FW_AIRSPD_MAX=20`. Ne pokreći Offboard,
excitation node ili direktne actuator komande.

## 3. Kopiranje novih logova

Poslije svakog leta pronađi najnoviji ULog:

```bash
find /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log \
  -type f -name '*.ulg' -printf '%T@ %p\n' | sort -nr | head
```

Kopiraj ih kao:

```text
validation_logs/standard_vtol_run_03_12ms.ulg
validation_logs/standard_vtol_run_04_18ms.ulg
```

Zatim za svaki pokreni `validate_standard_vtol_ulog.py` kao u prvom koraku.

## 4. Finalni train/validation split

Novi kandidat treniraj na brzinama 15 i 12 m/s, a validiraj na nezavisnim
letovima 15 i 18 m/s:

```bash
python3 tools/fit_standard_vtol_transition_model.py \
  --train \
    results/standard_vtol_run_01/samples.csv \
    results/standard_vtol_run_03_12ms/samples.csv \
  --validate \
    results/standard_vtol_run_02/samples.csv \
    results/standard_vtol_run_04_18ms/samples.csv \
  --output results/standard_vtol_parameter_fit_v2
```

## 5. Gate prije ugrađivanja u NMPC

Kandidat se ugrađuje u `StandardVtolTransitionRateModel` tek kada:

- validation acceleration RMSE bude manji od SDF baselinea na body-x i body-z;
- body-z RMSE bude ispod `0.5 m/s²` u stabilnom FW letu;
- znakovi `cx_alpha`, `cx_alpha2` i `cz_alpha` imaju fizički smisao;
- rezultat nije osjetljiv na zamjenu train/validation letova;
- hover model i dalje ima bias ispod `0.1 m/s²`;
- predikcija tokom obje tranzicije ima pravilan smjer, iako tranzicijski
  transienti nisu korišteni za fit.

Ako body-z ostane iznad `0.5 m/s²`, ne dodajemo još polinoma. Prvo se provjerava
airspeed/timing poravnanje i stvarna elevator joint dinamika. Tek nakon toga se
razmatra dodatno stanje za actuator lag ili lookup-table aerodinamika.

Konačni ground-truth rezultat i odbačeni kandidat su:

```text
results/standard_vtol_parameter_fit_v3_groundtruth/fit_report.md
results/standard_vtol_parameter_fit_v3_groundtruth/candidate_parameters.yaml
results/standard_vtol_parameter_fit_v3_groundtruth_run04/fit_report.md
```
