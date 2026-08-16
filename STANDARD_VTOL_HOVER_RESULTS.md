# Standard VTOL hover-Offboard checkpoint

Ovaj dokument bilježi granicu funkcionalnosti potvrđenu u PX4 SITL/Gazebo
testovima do 2026-08-16. Ne tvrdi da je VTOL tranzicija već implementirana.

## Šta radi

- PX4 odometry/status DDS komunikacija i frame konverzije;
- kontinuirani 20 Hz acados solve u shadow ili output režimu;
- hvatanje trenutne hover pozicije i yaw reference;
- jedna sekunda Offboard setpoint prestreama i bumpless handover;
- pusher zaključan na nulu tokom hover gatea;
- NMPC horizontalni/attitude-rate feedback;
- kritično prigušen hover-lift safety law oko ULog trim vrijednosti;
- automatski povratak u Position mode na timeout ili safety abort;
- status sa konfiguriranim timeoutom i stvarnim Offboard trajanjem.

## Razvojni gateovi

| Gate | Ishod | Ključni rezultat |
|---|---|---|
| 5 s | prošao | 4.97 s do timeouta, bez solver greške |
| pokušaj dužeg testa | prekinut | 6.36 s, `altitude_error`; otkrivena vertikalna oscilacija |
| 10 s, pokušaj 1 | prošao | 0.194 m max promjena visine, 0.067 m max horizontalno |
| 10 s, pokušaj 2 | prošao | 0.080 m max promjena visine, 0.128 m max horizontalno |
| 30 s | prošao | 30.02 s do timeouta, bez failsafea i solver greške |

## Prihvaćeni 30-sekundni rezultat

| Metrika | Vrijednost |
|---|---:|
| Maksimalna apsolutna promjena visine | 0.186 m |
| Maksimalni horizontalni pomak | 0.082 m |
| Maksimalna apsolutna vertikalna brzina | 0.097 m/s |
| Maksimalna horizontalna brzina | 0.090 m/s |
| Maksimalni roll/pitch nagib | 0.958 deg |
| Lift min/median/max | 0.5184/0.5198/0.5203 |
| Maksimalni roll/pitch/yaw rate setpoint | 0.0234/0.0247/0.0055 rad/s |
| PX4 failsafe | false |

Node log je prijavio `hover_test_timeout` nakon `30.02 s`, što je očekivani
završetak gatea. PX4 `vehicle_status` uzorci pokrivaju 29.484 s jer se status
loguje sporije od odometryja.

Veliki ULogovi nisu dio Git repozitorija. Četiri plant-identification ULoga i
kompaktni numerički arhivi prihvaćenih hover intervala ostaju lokalno u
ignorisanom `validation_logs/` direktoriju.

## Aktivne sigurnosne granice

- handover: horizontalna brzina ispod 0.5 m/s i vertikalna ispod 0.2 m/s;
- hover lift: 0.48–0.56, slew 0.10/s;
- pusher: uvijek 0 tokom hover gatea;
- rate setpoint: najviše 0.20/0.20/0.15 rad/s;
- abort: 0.5 m visinske greške, 0.75 m/s vertikalne brzine, 2 m/s
  horizontalne brzine, 5 m horizontalnog geofencea ili 25 deg nagiba;
- stale odometry, failsafe, izlazak iz MC konfiguracije ili tri solver greške
  takođe odmah vraćaju Position mode.

## Granica checkpointa i sljedeći eksperiment

Potvrđeno je stabilno zadržavanje hovera, ali nije potvrđeno praćenje pokretne
horizontalne reference niti VTOL mode switching. Sljedeći korak je poseban,
vremenski ograničen `2 m/s` MC horizontalni gate sa pusherom i dalje na nuli,
kontrolisanim ubrzanjem i obaveznim kočenjem nazad u hover. Tek nakon toga se
uvodi koordinisana PX4 VTOL transition komanda i trim raspored za 5–18 m/s.
