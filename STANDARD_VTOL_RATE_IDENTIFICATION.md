# Identifikacija rotacijske dinamike Standard VTOL-a

## Zašto je ova faza uvedena

Povučeni 10-state NMPC koristio je

```text
omega_B = omega_sp
```

unutar svakog predikcijskog koraka. Gate D ULogovi su pokazali da to nije
tačno pri gašenju lift motora i preuzimanju kontrole aerodinamičkim površinama.
Ova faza provjerava može li jednostavni closed-loop rate model zamijeniti tu
pretpostavku prije izgradnje novog OCP-a.

## Dataset i odvojena validacija

Alat `tools/extract_standard_vtol_rate_dataset.py` poravnava na 50 Hz:

```text
rate setpoint i measured body rate
logged angular acceleration
airspeed i VTOL state
MC/FW torque setpoint instance
lift motore, pusher i servo izlaze
attitude, body velocity i alpha/beta proxy
```

PX4 `NaN` izlaz za lift motor u armed FW stanju znači ugašen/nealociran motor
i prevodi se u nulu. `lift_fraction_proxy` je tačno `1` u MC, `0` u FW, a
stvarni omjer lift izlaza koristi se samo u stock transition blendu. To nije
buduća NMPC `lambda` komanda, nego dijagnostička rekonstrukcija.

Training skup:

- `standard_vtol_run_01.ulg` — stock 15 m/s;
- `standard_vtol_run_03_12ms.ulg` — stock 12 m/s;
- prihvaćeni stock Gate C;
- Gate D Attempt 06 i 07.

Netaknuti validation skup:

- `standard_vtol_run_02.ulg` — stock 15 m/s;
- `standard_vtol_run_04_18ms.ulg` — stock 18 m/s;
- Gate D Attempt 08, 09 i posljednji Attempt 10.

Validation logovi nisu korišteni za izbor delaya, parametara ni regularizacije.
Generisani CSV-ovi ostaju u ignorisanom `results/` direktoriju i ne ulaze u
git jer zauzimaju oko 63 MiB; svi se mogu ponovo generisati iz ULogova.

## Kandidat 1 — piecewise linearni rate model

Testirana jednačina je

```math
\omega_{k+1}=A_z\omega_k+B_z\omega_{sp,k-d}+c_z,
\qquad z\in\{MC,blend,FW\}.
```

Rezultat na validation skupu:

| Zona | one-step RMSE p/q/r [rad/s] | 0.5 s rollout p/q/r [rad/s] | Odluka |
|---|---:|---:|---|
| MC | 0.0047 / 0.0045 / 0.0006 | 0.0104 / 0.0170 / 0.0034 | PASS |
| blend | 0.0040 / 0.0171 / 0.0070 | 0.0179 / 0.0786 / 0.0823 | FAIL |
| FW | 0.0062 / 0.0061 / 0.0005 | 0.0607 / 0.0446 / 0.0070 | FAIL |

Blend model je lošiji od zero-order-hold baselinea, a FW `A` ima pol veći od
jedan. Kandidat se ne ugrađuje u NMPC.

## Kandidat 2 — airspeed/allocation-scheduled LPV

Testirana je bogatija jednačina

```math
\dot\omega=K(V,1-\lambda)(\omega_{sp}-\omega)
+b(V,\lambda,\alpha,\beta,\theta,phase).
```

Ona poboljšava blend pitch one-step grešku na `0.0103 rad/s`, ali 0.5 s
rollout ostaje `0.0401 / 0.0781 / 0.0748 rad/s`. Najveći lokalni diskretni
spectral radius je `1.102` u blendu i `1.133` u FW. Povećanje ridge
regularizacije od `0.001` do `1.0` nije uklonilo nestabilnost, pa problem nije
samo numerička prenaučenost.

**IDENTIFICATION_GATE=FAIL za oba čista first-order closed-loop kandidata.**

## Fizički zaključak

U FW ulasku nije dovoljno reći da body rate kasni za setpointom. PX4 rate
regulator generiše ograničeni torque zahtjev, allocator ga pretvara u elevone,
a stvarni aerodinamički moment zavisi od airspeeda, angle of attacka i
defleksije površina. U Gate D logovima pozitivan `q_sp` istovremeno postoji sa
velikim negativnim stvarnim `q`; običan pozitivan first-order gain ne može
objasniti taj transient bez eksplicitnog momenta i saturacije.

Zato novi model koristi rigid-body ugaonu jednačinu:

```math
\dot\omega_B=J^{-1}\left(
\tau_{MC}+\tau_{FW}+\tau_{aero}
-\omega_B\times J\omega_B\right),
```

sa

```text
tau_MC = lambda * allocator_MC(PID_MC(omega_sp - omega))
tau_FW = (1-lambda) * allocator_FW(PID_FW(omega_sp - omega), airspeed)
```

i identificiranim efektivnim aerodinamičkim momentom površina. PID integrator se inicijalizira iz PX4
statusa i unutar kratkog horizonta može biti bounded parametar; ne moramo zbog
njega odmah dodati pojedinačne RPM stateove.

## Implementirani torque-informed lanac

NumPy model sada reproducira:

```text
rate_sp -> PX4 MC/FW PID+FF -> control allocator
        -> lift motori / tri servo komande -> rigid-body moment
```

PX4 rate-controller reprodukcija na FW logu ima pitch torque RMSE `0.0052`, a
sa tačnim 1 s filtriranjem calibrated airspeeda `0.0106` kroz transition blend.
Allocator iz trening ULoga predviđa četiri lift-motor komande na netaknutom
logu sa RMSE manjim od `0.002`; surface mapping odgovara PX4 Standard VTOL
geometriji unutar `0.31 deg`.

Gazebo `JointPositionController` ne postavlja zglob trenutno. ULog
identifikacija, sa lijevim/desnim elevonom prisilno jednakim, daje:

| Površina | time constant [s] | static gain |
|---|---:|---:|
| lijevi/desni elevon | 1.0289 | 1.3140 |
| elevator | 0.7224 | 1.1489 |

Na odvojenom validation letu taj lag smanjuje roll angular-acceleration RMSE
sa `4.230` na `0.151 rad/s^2`. Zbog sporog odziva model je proširen sa 13
rigid-body varijabli na tri surface-angle stanja: ukupno **16 state varijabli**.
Motor RPM i dalje nije OCP stanje.

Čisti SDF pitch moment je preosjetljiv na malu grešku skrivenog elevator
zgloba. Zato je na trening letu identificiran dimenzionalni model

```math
M_y=\bar q(c_0+c_\alpha\alpha+c_{\alpha2}\alpha|\alpha|
          +c_q q/V+c_{\delta_e}\delta_e).
```

Na netaknutom validation letu FW pitch-moment RMSE pada sa `0.478` na
`0.098 Nm` za FW fit; stabilni zone-balanced model smanjuje blend RMSE sa
`0.421` na `0.236 Nm`. Direktni blend-only fit je odbijen iako ima bolji
pointwise correlation: naučio je aerodinamičko anti-prigušenje i postao
nestabilan u 0.5 s rolloutu.

Strogi 0.5 s test ne koristi budući logged torque ni servo izlaz. Rezultat je:

| Zona | rate-sp model p/q/r RMSE [rad/s] | ZOH p/q/r | Odluka |
|---|---:|---:|---|
| MC | 0.0053 / 0.0075 / 0.0019 | 0.0436 / 0.0525 / 0.0097 | PASS |
| blend | 0.0277 / 0.1104 / 0.0342 | 0.2410 / 0.1234 / 0.0648 | FAIL |
| FW | 0.0288 / 0.0624 / 0.0208 | 0.1552 / 0.0808 / 0.0275 | blizu, FAIL |

Plant-only dijagnostika sa budućim logged actuator komandama daje blend pitch
RMSE `0.4602 rad/s`, pa preostali blocker nije rate PID/allocator nego
efektivni mixed-authority pitch plant. Nema novog leta dok blend rollout ne
prođe.

## Sljedeći implementacijski gate

NumPy 16-state rotational validation model postoji, još bez acadosa:

1. reproducirati MC i FW PX4 rate PID/FF i saturacije iz aktivnih parametara;
2. algebraički mapirati MC torque na lift motore i FW torque na tri površine;
3. koristiti SDF motor/force model, identificirani surface lag i efektivni
   pitch-moment model;
4. prvo validirati `logged torque -> angular acceleration` da se izoluje plant;
5. zatim validirati `rate_sp -> predicted torque -> angular acceleration`;
6. tek nakon pravilnog pitch-transient znaka prenijeti jednačinu u CasADi.

Acceptance prije OCP-a:

```text
pravilan znak q i q_dot kroz svaki FW ulazak
stabilan 0.5 s rollout u MC, blend i FW
blend pitch-rate rollout RMSE <= 0.05 rad/s
FW pitch-rate rollout RMSE <= 0.05 rad/s
bez skrivenog korištenja validation torque/servo komande kao budućeg inputa
```

## Reprodukcija

Za svaki ULog:

```bash
/usr/bin/python3 tools/extract_standard_vtol_rate_dataset.py LOG.ulg \
  --output results/standard_vtol_rate_identification/datasets/IME
```

Piecewise kandidat pokreće `tools/fit_standard_vtol_rate_dynamics.py`, a LPV
kandidat `tools/fit_standard_vtol_rate_lpv.py`; oba zahtijevaju eksplicitne
`--train`, `--validate` i `--output` argumente. Tačan split je gore zapisan da
rezultat ostane ponovljiv.

Torque-informed provjere su:

```bash
python tools/validate_standard_vtol_rate_controller.py TRAIN.csv VALIDATE.csv \
  --output results/rate_controller_check

python tools/identify_standard_vtol_surface_dynamics.py \
  --train TRAIN.csv --validate VALIDATE.csv --output results/surface_dynamics

python tools/identify_standard_vtol_pitch_moment.py \
  --train TRAIN.csv --validate VALIDATE.csv --output results/pitch_moment

python tools/validate_standard_vtol_rotational_replay.py TRAIN.csv VALIDATE.csv \
  --output results/rotational_replay

python tools/validate_standard_vtol_rotational_rollout.py TRAIN.csv VALIDATE.csv \
  --zones blend fw --output results/rotational_rollout
```
