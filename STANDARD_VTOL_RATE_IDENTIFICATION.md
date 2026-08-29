# Identifikacija rotacijske dinamike Standard VTOL-a

## Zašto je ova faza uvedena

Povučeni 10-state NMPC koristio je

```text
omega_B = omega_sp
```

unutar svakog predikcijskog koraka. Gate D ULogovi su pokazali da to nije
tačno pri gašenju lift motora i preuzimanju kontrole aerodinamičkim površinama.
Ova faza provjerava može li jednostavni closed-loop rate model zamijeniti tu
pretpostavku prije izgradnje novog 13-state OCP-a.

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

i SDF aerodinamičkim momentom površina. PID integrator se inicijalizira iz PX4
statusa i unutar kratkog horizonta može biti bounded parametar; ne moramo zbog
njega odmah dodati pojedinačne RPM stateove.

## Sljedeći implementacijski gate

Prvo se pravi NumPy 13-state rotational validation model, još bez acadosa:

1. reproducirati MC i FW PX4 rate PID/FF i saturacije iz aktivnih parametara;
2. algebraički mapirati MC torque na lift motore i FW torque na tri površine;
3. koristiti postojeći SDF `motor_wrench()` i `aerodynamic_wrench()`;
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
