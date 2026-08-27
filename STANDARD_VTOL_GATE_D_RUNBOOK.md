# Gate D — prva NMPC-koordinisana VTOL tranzicija

Gate D izvodi jednu automatsku sekvencu:

```text
MC hover -> 8 m/s -> PX4 front transition -> kratki FW hold
         -> PX4 back transition -> kočenje -> MC hover
```

NMPC šalje collective, pusher i body-rate setpointe. PX4 i dalje posjeduje
VTOL state machine, airspeed blend, control allocation i unutrašnji rate loop.
NMPC ne komanduje pojedinačne motore. Ovo je prvi Gate koji mijenja VTOL state;
ne pokretati ga bez najmanje 30 m visine i 500 m čistog prostora ispred nosa.

Gate D zahtijeva custom PX4 `nmpc-external-pusher` build koji ažurira Standard
VTOL transition timer i MC/FW weights u Offboard body-rate režimu. NMPC šalje
raw collective; PX4 je jedini vlasnik lift blenda.

## Acceptance envelope

```text
Gate timeout:              90 s PX4 vremena
MC transition trigger:     ground speed i CAS >= 7.5 m/s tokom 1 s
front-transition timeout:  12 s
FW hold nakon FW ulaska:   5 s
referentna brzina:         0 -> 8 -> 12 -> 0 m/s
pusher command:            MC <= 0.30; front transition <= 0.40
apsolutna greška visine:   <= 2.0 m
apsolutni roll/pitch:      <= 20 deg
brzina:                    <= 14 m/s
cross-track:               <= 20 m
solver failures:           0
završno stanje:            MC, pusher 0, speed <= 0.5 m/s tokom 2 s
```

Na grešku se ne bira odmah Position dok je vozilo FW. Node prvo traži PX4 back
transition i zadržava konzervativan setpoint. Tek nakon potvrđenog MC stanja
traži Position mode. Ako automatika očigledno ne uspijeva, ručni prioritet je
`Transition to Multicopter`, pa tek kada je vozilo MC `Position`.

## Jednokratno prije prvog leta

```bash
cd /home/imran/Repositories/px4-mpc
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --packages-select px4_mpc --symlink-install
```

Build mora završiti bez greške. Svaki Terminal ispod otvori kao novi shell.

Offline Gate D sa PX4-owned lift blendom je prošao (`53.05 s`, `12.926 m/s`,
visina `1.029 m`, tilt `5.93 deg`, stvarni peak pusher `0.443`, solver
failures `0`, završni MC). Regresija se može ponoviti:

```bash
cd /home/imran/Repositories/px4-mpc
source .venv/bin/activate
source scripts/source_ros2_nmpc.bash
python3 tools/simulate_transition_gate_d.py
```

Mora završiti sa `transition_gate_d_offline=PASS`.

## Terminal 1 — PX4/Gazebo

Ne sourceati ROS ni `px4-mpc/.venv` u ovom terminalu:

```bash
cd /home/imran/Repositories/PX4-Autopilot
git branch --show-current
git log -1 --oneline
make px4_sitl gz_standard_vtol
```

Branch mora biti `nmpc-external-pusher`.
Commit mora biti `7558a3d188 fix(vtol): update standard transition in offboard
rates` ili njegov potomak. U `pxh>` shellu:

```text
param set VT_EXT_PUSH_EN 1
param set VT_EXT_PUSH_MAX 0.40
param set VT_EXT_PUSH_SLEW 0.33
param show VT_EXT_PUSH_EN
param show VT_EXT_PUSH_MAX
param show VT_EXT_PUSH_SLEW
```

Mora prikazati `1`, `0.40`, `0.33`. Ne mijenjati PX4 stock transition
parametre `VT_ARSP_BLEND`, `VT_ARSP_TRANS`, `VT_TRANS_MIN_TM`,
`VT_B_TRANS_RAMP` ili `VT_B_TRANS_DUR`.

## Terminal 2 — DDS Agent

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/setup_standard_vtol_nmpc.bash
microxrce_agent_install/bin/MicroXRCEAgent udp4 -p 8888
```

Ostaviti terminal otvoren.

## Terminal 3 — Gate D node

```bash
cd /home/imran/Repositories/px4-mpc
source scripts/source_ros2_nmpc.bash
ros2 launch px4_mpc standard_vtol_nmpc_launch.py \
  allow_offboard_output:=true \
  allow_external_pusher_output:=true \
  allow_transition_gate_d_output:=true \
  transition_gate_d_max_seconds:=90.0 \
  transition_gate_d_pusher_max:=0.40
```

Startup poruka mora sadržati `guarded Gate D front/back transition`. Ostaviti
terminal otvoren. Warning za Matplotlib `Axes3D` nije razlog za prekid.

## QGC — priprema leta

1. Vozilo mora biti MC i Position mode.
2. Armirati i podići na najmanje 30 m iznad tla.
3. Nos usmjeriti prema najmanje 500 m praznog prostora.
4. Držati stabilan hover najmanje 10 s.
5. Ne pritiskati QGC transition dugme tokom normalnog Gate D testa.
6. Biti spreman za ručni `Transition to Multicopter` ako recovery zakaže.

Negativan CAS u stacionarnom hoveru je normalan ako status kaže
`available=True`. Gate D traži pozitivan CAS tek nakon početka ubrzavanja.

## Terminal 4 — pokretanje i automatsko praćenje

```bash
cd /home/imran/Repositories/px4-mpc
bash scripts/run_transition_gate_d.bash
```

Kada skripta traži potvrdu, provjeri PX4 parametre, visinu, smjer i prostor, pa
upiši tačno:

```text
GATE-D-READY
```

Normalan redoslijed statusa je:

```text
mc_accelerate -> front_transition -> fw_hold
              -> back_transition -> mc_recovered -> complete
```

Terminal 4 tokom front tranzicije prikazuje i `lift_weight` i `collective`.
U `mc_accelerate` oba ostaju približno na hover vrijednosti (`1.0` i `0.52`).
Nakon najmanje `2 s` u `front_transition`, `lift_weight` mora pasti kada CAS
pređe `8 m/s`. Prikazani `collective` je sada raw NMPC zahtjev i treba ostati
približno u hover rasponu `0.48..0.56`; PX4 interno množi taj zahtjev svojim
weightom. Stvarno rasterećenje lift-motora potvrđuje se iz ULoga, ne padom raw
collectivea u ROS statusu.

Attempt 06 je prvi put dokazao cijeli PX4 slijed `3 -> 1 -> 4 -> 2 -> 3`,
ali je stari forsirani front-pusher `0.60` doveo do FW ulaska pri CAS
`13.27 m/s` i trenutnog prelaska sigurnosnog limita `14 m/s`. Zato ovaj gate
sada koristi plafon `0.40`, ne forsira pusher kada je brzina iznad reference i
smanjuje ga sa `0.33/s`. Ne vraćati `0.60` radi bržeg ulaska u tranziciju.

Attempt 07 je ušao u FW pri CAS `10.59 m/s`, ali je dotadašnji pitch-rate
governor ostao na MC limitu `0.10 rad/s`. Stock PX4 ULog u istom trenutku
koristi približno `0.44..0.55 rad/s`; uski limit nije mogao zaustaviti nagli
pitch transient nakon gašenja lift-motora. Od ovog checkpointa `fw_hold`,
`back_transition` i abort-recovery koriste stock-informisan limit `0.65
rad/s`, slew `1.50 rad/s^2` i aktivno nivelisanje. `front_transition` prije FW
potvrde ostaje na blagom, ranije validiranom `0.10 rad/s` limitu.

Prikaz `pitch=actual/reference` koristi interni FLU znak. Tokom
`front_transition` referenca je `0 deg`, prema uspješnom stock PX4 ULogu, a
stvarni pitch treba ostati unutar približno `+/-4 deg`. Automatski recovery
sada počinje najkasnije na `10 deg` tilta ili `1.5 m/s` vertikalne brzine u toj
fazi. Ne širiti te granice radi dobijanja PASS-a.

U `mc_accelerate` je dozvoljen najviše `0.45 s` izolovanog DDS odometry
razmaka, jer Attempt 04 ULog dokazuje da je pri izmjerenom ROS razmaku
`0.365 s` PX4 local-position ostao kontinuiran na `<=0.032 s`. Od
`front_transition` nadalje ostaju strogi wall/PX4 age limiti `0.30/0.20 s`.

FW krug nije dio ovog testa. Vozilo treba nastaviti približno ravno i nakon 5 s
FW stanja automatski zatražiti back transition. Ako počne kružiti, to obično
znači da je napustilo Offboard; node tada mora prijaviti recovery. Ne pokretati
novi gate u istom letu.

PASS završava sa:

```text
abort_reason=gate_d_complete
gate_d=[state=complete,vtol_state=3,...front_ack=True,back_ack=True,...]
solver_failures=0
total_solver_failures=0
ROS_GATE_D=PASS
```

Svaki drugi `abort_reason` je FAIL. Ne ponavljati naslijepo.

## Nakon leta — ULog

Sletjeti, disarmirati i ugasiti PX4/Gazebo. Pronaći samo najnoviji log:

```bash
find /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log \
  -type f -name '*.ulg' -printf '%T@ %s %p\n' | sort -nr | head -1
```

Zamijeniti putanju u narednoj komandi stvarnim rezultatom:

```bash
cd /home/imran/Repositories/px4-mpc
/usr/bin/python3 tools/analyze_transition_shadow_ulog.py --gate-d \
  /home/imran/Repositories/PX4-Autopilot/build/px4_sitl_default/rootfs/log/YYYY-MM-DD/HH_MM_SS.ulg
```

ULog je prihvaćen tek kada završava sa:

```text
complete_state_sequence=PASS
altitude_loss=PASS
tilt=PASS
no_failsafe=PASS
nmpc_external_pusher_enabled=PASS
pusher_bounded=PASS
lift_blend_observed=PASS
transition_gate_d_ulog=PASS
```

Ne kopirati cijeli ULog u git repo. Nakon PASS-a zapisujemo njegovu putanju,
SHA-256 i mali tekstualni sažetak; stari neuspješni SITL logovi se mogu obrisati
tek nakon što potvrdimo koji je Gate D log.
