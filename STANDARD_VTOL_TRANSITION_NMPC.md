# Standard VTOL transition NMPC

## Objective

Control the PX4 Gazebo `standard_vtol` from multicopter hover through the
transition and into steady forward flight. The first implementation keeps PX4's
body-rate controller and control allocator in the loop. The NMPC commands
collective lift thrust, pusher thrust, and body rates; it does not directly
command individual motors or elevons.

This architecture is intentional. Direct actuator NMPC would also have to
replace PX4's rate stabilization, allocation, transition state machine, and
failsafes before the outer-loop model has been validated.

## Source of truth

The plant parameters come from:

```text
PX4-Autopilot revision: 5f1eae330b
Tools/simulation/gz/models/standard_vtol/model.sdf
ROMFS/px4fmu_common/init.d-posix/airframes/4004_gz_standard_vtol
```

The implementation is in
`px4_mpc/models/standard_vtol_gz_model.py` and contains two models:

- `StandardVtolGazeboModel`: 18-state, actuator-level 6-DoF validation plant.
- `StandardVtolTransitionRateModel`: 10-state model for the first NMPC.

### Validation plant

State:

```text
x = [p_W(3), v_W(3), q_WB(4), omega_B(3), Omega_rotors(5)]
```

Input:

```text
u = [motor_0..motor_4, left_elevon, right_elevon, elevator]
```

The plant includes:

- composite mass and inertia;
- asymmetric first-order motor dynamics;
- quadratic thrust and reaction torque;
- rotor translational drag and rolling moment;
- the three Gazebo LiftDrag surfaces, including stall and control deflection;
- the real SDF rotor locations, axes, limits, and coefficients.

It omits contacts, sensors, and the negligible gyroscopic effect of the rendered
rotor links. These omissions do not change the free-flight model needed by the
NMPC.

### NMPC prediction model

State:

```text
x_mpc = [p_W(3), v_W(3), q_WB(4)]
```

Input:

```text
u_mpc = [collective_lift, pusher, body_rate_x, body_rate_y, body_rate_z]
```

PX4 realizes the requested rates with the lift motors and elevons. The
prediction model retains rotor thrust, rotor drag, and wing lift/drag, but does
not predict the fast rate-loop dynamics. This produces a smaller and better
conditioned optimal-control problem.

## Mathematical model

### Frames and notation

The equations use Gazebo coordinates:

- `W`: world ENU frame (`x` east, `y` north, `z` up);
- `B`: body FLU frame (`x` forward, `y` left, `z` up);
- `R_WB(q)`: rotation matrix that maps a vector from `B` to `W`;
- `q_WB = [q_w, q_x, q_y, q_z]`: scalar-first unit quaternion;
- `p_W`, `v_W`: position and velocity expressed in `W`;
- `omega_B = [p, q, r]`: body angular velocity expressed in `B`;
- `m`: total vehicle mass and `J`: inertia matrix about the center of mass.

PX4 topics normally use NED/FRD coordinates. Conversion between PX4 and this
model must happen at the ROS interface; the dynamics below must not mix the two
frame conventions.

### Full 18-state Gazebo-derived plant

The complete state and actuator input are

```text
x_full = [p_W(3), v_W(3), q_WB(4), omega_B(3), Omega(5)] in R^18
u_full = [c_0, c_1, c_2, c_3, c_p, delta_l, delta_r, delta_e] in R^8
```

Here `Omega_i` is actual rotor speed, `c_i` is normalized motor command,
`c_p` is the pusher command, and `delta` values are surface joint angles.

The rigid-body equations are

```math
\dot p_W = v_W,
```

```math
\dot v_W = \frac{1}{m}R_{WB}(q)
            \left(F_{mot,B}+F_{aero,B}\right)
            + \begin{bmatrix}0&0&-g\end{bmatrix}^{T},
```

```math
\dot q_{WB} = \frac{1}{2}q_{WB}\otimes
               \begin{bmatrix}0&p&q&r\end{bmatrix}^{T},
```

```math
\dot\omega_B = J^{-1}\left(
    \tau_{mot,B}+\tau_{aero,B}
    -\omega_B\times(J\omega_B)\right).
```

The implementation of these four equations is the `derivative()` method of
`StandardVtolGazeboModel`.

#### Motor dynamics and wrench

Normalized command is converted to demanded rotor speed with

```math
\Omega_{i,cmd}=\Omega_{i,min}
 +\operatorname{sat}(c_i,0,1)(\Omega_{i,max}-\Omega_{i,min}).
```

Gazebo applies an asymmetric first-order motor response:

```math
\dot\Omega_i=\frac{\Omega_{i,cmd}-\Omega_i}{\tau_i},
\qquad
\tau_i=\begin{cases}
0.0125\;s,&\Omega_{i,cmd}>\Omega_i,\\
0.025\;s,&\Omega_{i,cmd}\leq\Omega_i.
\end{cases}
```

For rotor axis `a_i`, position `r_i` relative to the center of mass, and
turning-direction sign `d_i`, the principal thrust and reaction torque are

```math
T_i=k_{T,i}\Omega_i^2,
\qquad F_{T,i}=T_i a_i,
```

```math
\tau_i=r_i\times F_i-d_i k_{M,i}T_i a_i+\tau_{roll,i}.
```

The Gazebo rotor-drag terms are

```math
v_{i,B}=v_B+\omega_B\times r_i-v_{wind,B},
```

```math
v_{i,\perp}=v_{i,B}-(v_{i,B}^{T}a_i)a_i,
```

```math
F_{drag,i}=-|\Omega_i|k_{D,i}v_{i,\perp},
\qquad
\tau_{roll,i}=-|\Omega_i|k_{R,i}v_{i,\perp},
```

so that `F_i = F_T,i + F_drag,i`. The total motor wrench is

```math
F_{mot,B}=\sum_{i=0}^{4}F_i,
\qquad
\tau_{mot,B}=\sum_{i=0}^{4}\tau_i.
```

These equations are implemented in `motor_speed_derivative()` and
`motor_wrench()`.

#### Wing and control-surface aerodynamics

For each of the left wing, right wing, and elevator, air velocity at the center
of pressure is

```math
v_{cp,B}=v_B+\omega_B\times r_{cp}-v_{wind,B}.
```

After removing the component along the wing span, its magnitude gives dynamic
pressure

```math
\bar q=\frac{1}{2}\rho\|v_{LD}\|^2.
```

Before stall, Gazebo uses

```math
C_L=C_{L_\alpha}\alpha\cos^2\Lambda+C_{L_\delta}\delta,
```

```math
C_D=\left|C_{D_\alpha}\alpha\cos^2\Lambda\right|,
\qquad
C_M=C_{M_\alpha}\alpha\cos^2\Lambda.
```

`Lambda` is sweep angle. After `|alpha| > alpha_stall`, the model switches to
the `cla_stall`, `cda_stall`, and `cma_stall` slopes from the SDF. The forces
and moment are

```math
F_L=C_L\bar qS e_L,
\qquad F_D=C_D\bar qS e_D,
```

```math
F_{aero,B}=\sum_j(F_{L,j}+F_{D,j}),
```

```math
\tau_{aero,B}=\sum_j\left[
    r_{cp,j}\times(F_{L,j}+F_{D,j})
    +C_{M,j}\bar q_jS_j e_{span,j}\right].
```

The piecewise stall law, directions of lift and drag, and surface limits are
implemented in `surface_wrench()` and `aerodynamic_wrench()`.

### The transition model is 10-state, not 10-DoF

The aircraft always has six physical degrees of freedom:

```text
translation: x, y, z                         -> 3 DoF
rotation:    roll, pitch, yaw                -> 3 DoF
                                                ------
                                                6 DoF
```

“10-state” means that ten numbers are used to represent the NMPC state:

```text
x_mpc = [p_x, p_y, p_z, v_x, v_y, v_z,
         q_w, q_x, q_y, q_z] in R^10
```

The quaternion needs four numbers to represent three rotational degrees of
freedom and must satisfy

```math
q_{WB}^{T}q_{WB}=1.
```

Consequently, 10 state variables do not imply 10 physical DoF.

The NMPC inputs are

```text
u_mpc = [c_lift, c_pusher, p_cmd, q_cmd, r_cmd] in R^5.
```

The reduced transition equations are

```math
\dot p_W=v_W,
```

```math
\dot v_W=\frac{1}{m}R_{WB}(q)
  \left(F_{lift}(c_{lift})+F_{pusher}(c_{pusher})
  +F_{rotor\ drag}+F_{aero}\right)+g_W,
```

```math
\dot q_{WB}=\frac{1}{2}q_{WB}\otimes
 \begin{bmatrix}0&p_{cmd}&q_{cmd}&r_{cmd}\end{bmatrix}^{T}.
```

Compared with the 18-state plant, the following states and equations are
removed from the online optimizer:

| Removed from NMPC state | Replacement assumption |
|---|---|
| `omega_B(3)` | PX4's fast rate controller tracks `[p_cmd,q_cmd,r_cmd]` |
| `Omega(5)` | rotor speeds follow the demanded normalized commands rapidly |
| angular rigid-body equation | PX4 rate controller and allocator generate moments |
| individual lift motor commands | PX4 allocator distributes collective lift |
| elevon deflections | PX4 uses surfaces to realize rate commands during transition |

The NMPC still predicts all six vehicle motions because position, velocity, and
orientation remain in the state. It simply delegates fast rotational and
actuator dynamics to PX4. This is a cascaded controller:

```text
NMPC (20 Hz)
  -> collective lift + pusher + desired body rates
  -> PX4 rate controller and control allocator
  -> motors and elevons
  -> Gazebo 6-DoF aircraft
```

This reduction is appropriate for the first transition controller only if the
PX4 rate loop tracks substantially faster than the NMPC sampling period. Model
validation must check that assumption. If it is inadequate, the next model adds
`omega_B` and first-order rate-tracking dynamics before considering direct
actuator NMPC.

## Sanity checks from the Gazebo model

The actuator-level hover allocation is approximately:

```text
motor 0: 0.519955
motor 1: 0.520284
motor 2: 0.519955
motor 3: 0.520284
```

The small difference balances the composite center-of-mass offset. The
collective approximation is `0.520120`.

A zero-pitch, neutral-surface trim provides the following NMPC initial guesses:

| Airspeed | Lift command | Pusher command | Wing lift |
|---:|---:|---:|---:|
| 0 m/s | 0.5201 | 0.0000 | 0.0 N |
| 5 m/s | 0.4975 | 0.0755 | 4.1 N |
| 10 m/s | 0.4226 | 0.1510 | 16.6 N |
| 12 m/s | 0.3719 | 0.1813 | 23.8 N |
| 15 m/s | 0.2538 | 0.2266 | 37.2 N |
| 17 m/s | 0.0838 | 0.2568 | 47.8 N |

Above roughly 17 m/s, zero pitch creates more lift than weight. The final trim
generator must therefore optimize pitch/angle of attack rather than clamp lift
motors and keep the body level.

## Implementation path

### 1. Validate the plant against Gazebo

Run only normal PX4-controlled SITL flights. Record vehicle pose, velocity,
attitude, angular velocity, actuator motor commands, servo commands, and
airspeed. Replay those actuator inputs through `StandardVtolGazeboModel` and
compare one-step acceleration and angular-acceleration predictions.

The exact PX4/QGroundControl flight, ULog and validator procedure is documented
in [`STANDARD_VTOL_PLANT_VALIDATION.md`](STANDARD_VTOL_PLANT_VALIDATION.md).

Acceptance gate:

- correct force and moment signs on all axes;
- hover acceleration bias below `0.1 m/s^2`;
- forward-flight acceleration error below `15%` over short windows;
- quaternion and ENU/FLU to NED/FRD transforms covered by tests.

No Offboard actuator test is needed for this phase.

Status: passed for translational dynamics using the Gazebo ground-truth fields
already stored in the four ULogs. On the independent 15 and 18 m/s validation
runs, the original SDF-derived model achieves body-frame acceleration RMSE
`[0.097, 0.064, 0.431] m/s^2`. An empirical polynomial refit performed worse
and was rejected. Rotational moment residuals do not block the first reduced
NMPC because PX4 retains the closed body-rate loop.

### 2. Generate a trim corridor

Solve a steady-flight nonlinear program for airspeeds from 0 to 22 m/s. Its
decision variables are pitch, lift command, pusher command, and optionally
elevator trim. Constrain translational and angular accelerations to zero. Use
the `nominal_level_flight_trim()` values as initial guesses.

The result must be a smooth table:

```text
V -> [pitch_ref, lift_ref, pusher_ref, elevator_ref]
```

This table becomes the transition reference and warm start for the online
solver.

Status: implemented in `StandardVtolTrimSolver` and
`tools/generate_standard_vtol_trim_corridor.py`. The generated 0–22 m/s table
is force-balanced to below `1e-8 m/s^2`. From 5 m/s upward, elevator trim is
also solved so pitch angular acceleration is zero. Below 5 m/s the multicopter
allocator is assumed to supply the small balancing moment. Lift command is
constrained to decrease monotonically and reaches the wing-borne branch at 10
m/s. Elevator is reported for verification; it remains under PX4's rate loop
and is not a direct NMPC input.

Regenerate it with:

```bash
MPLCONFIGDIR=/tmp/matplotlib-px4-mpc python3 \
  tools/generate_standard_vtol_trim_corridor.py \
  --output results/standard_vtol_trim_corridor
```

### 3. Build the CasADi/acados NMPC

Use the reduced rate model first:

- sample time: `0.05 s`;
- horizon: begin with `N=30` (`1.5 s`), then extend if timing allows;
- discretization: RK4 or acados ERK;
- online parameters: wind estimate and reference trim point;
- warm start: shifted previous solution plus the trim corridor.

Cost terms:

- altitude and vertical speed;
- forward airspeed;
- lateral position/velocity;
- attitude error and body-rate command;
- distance from the trim lift/pusher schedule;
- input slew rate, especially lift-motor reduction and pusher ramp.

Initial conservative constraints:

```text
0.10 <= collective_lift <= 0.65
0.00 <= pusher <= 0.60
abs(roll) <= 20 deg
-20 deg <= pitch <= 15 deg
abs(body_rate_xy) <= 0.5 rad/s
abs(yaw_rate) <= 0.3 rad/s
altitude >= configured floor
```

Add constraints on command rate so the optimizer cannot reproduce the previous
constant-thrust runaway.

Status: CasADi prediction model and the acados OCP are implemented in
`px4_mpc/models/standard_vtol_casadi_model.py`. It uses exactly the same motor,
rotor-drag and scheduled-elevator aerodynamic equations as the validated NumPy
model. `px4_mpc/controllers/standard_vtol_nmpc.py` supplies the 30-step,
1.5-second OCP. Equivalence tests cover hover and representative transition
states, including wind and elevator trim, and pass to numerical precision.
`tools/simulate_standard_vtol_nmpc.py` applies the same external command slew
limiter that protects the ROS output.

Offline 10 and 15 m/s runs both completed with zero solver failures, maximum
altitude error `0.055 m`, and final speed equal to the reference. The measured
p99 solve times were `6.54 ms` and `5.56 ms`, respectively.

Run the model-equivalence tests with:

```bash
cd /home/imran/Repositories/px4-mpc
PYTHONPATH=px4_mpc .venv/bin/python -m unittest discover \
  -s px4_mpc/test -p 'test_*.py'
```

No PX4, QGroundControl, ROS 2 or new flight log is required for this step.

### 4. Shadow-mode SITL

Run the NMPC at 20 Hz while PX4 remains in Position mode. Publish predictions
and proposed controls to diagnostic topics only. Do not publish to
`/fmu/in/vehicle_rates_setpoint`.

Acceptance gate:

- solve time below 40 ms at the 99th percentile;
- no solver failures during a complete PX4 transition;
- predicted vertical speed and airspeed have the correct trend;
- commands remain inside all bounds with smooth lift-to-pusher transfer.

Status: the ROS 2 node and launch file are implemented. It always calculates
and publishes `/standard_vtol_nmpc/proposed_control`; its default launch mode
cannot publish PX4 setpoints. Live shadow checks were completed before guarded
output was enabled; the next shadow requirement applies to the new moving
horizontal reference before it may command PX4.

### 5. Hover-only Offboard

Enable the NMPC output only near hover, with pusher fixed to zero. The setpoint
must be initialized from the current position and attitude, not a hard-coded
origin. A watchdog must request PX4 Position mode when any of these occurs:

- stale odometry or airspeed;
- solver failure or deadline miss;
- altitude, tilt, speed, or geofence violation;
- loss of Offboard acceptance.

Unlike the old excitation node, abort must change PX4 mode and stop applying
open-loop thrust.

Status: guarded hover-only output, one-second setpoint prestream, explicit
enable/disable services, slew limiting and watchdog fallback to Position mode
are implemented. The live SITL acceptance test is documented in
`STANDARD_VTOL_OFFBOARD_RUNBOOK.md`. Five- and ten-second development gates
passed, followed by the final 30-second gate on 2026-08-16. The 30-second run
ended only at its configured timeout, with `0.186 m` maximum altitude change,
`0.082 m` maximum horizontal displacement, `0.097 m/s` maximum vertical speed,
`0.958 deg` maximum tilt, no failsafe, and no solver failure. Full evidence is
recorded in `STANDARD_VTOL_HOVER_RESULTS.md`.

### 6. Incremental transition tests

First add a `2 m/s` horizontal-speed gate while PX4 remains in multicopter
configuration. This validates reference motion, heading projection, position
envelope, and braking without mixing those problems with VTOL mode switching.
Then increase the commanded airspeed in separate coordinated runs: 5, 10, 12,
15, then 18 m/s. At each stage validate altitude error, model residuals, solver
timing, and control smoothness. Only after those gates pass should a continuous
hover-to-forward reference be attempted.

PX4's VTOL state and control allocation must remain authoritative in this
version. Log `VtolVehicleStatus` and coordinate the transition command with the
NMPC trim schedule. Do not simultaneously run an independent hard-coded PX4
transition ramp and an NMPC ramp without explicitly defining which layer owns
lift-motor shutdown.

### 7. Full forward-flight tracking

After transition is repeatable, add forward-flight position/path tracking and
wind estimation. Direct elevon or motor allocation is a later optimization,
not a prerequisite for a functioning transition NMPC.

## Immediate next code milestone

Implementation checklist:

1. [x] an ENU/FLU to PX4 NED/FRD conversion module with tests;
2. [x] a steady-flight trim solver using this model;
3. [x] a CasADi version of `StandardVtolTransitionRateModel`;
4. [x] an acados OCP with the constraints above;
5. [x] an offline closed-loop transition simulation;
6. [x] a ROS 2 node that starts in mandatory shadow mode;
7. [x] pass the five-second live SITL hover-only Offboard acceptance test;
8. [x] pass the 30-second hover-hold Offboard acceptance test;
9. [x] pass the implemented bounded 2 m/s multicopter horizontal-speed gate
   documented in `STANDARD_VTOL_MC_FORWARD_RUNBOOK.md`;
10. [x] validate the custom PX4 pusher path with a bounded `0 -> 0.05 -> 0`
    motor-5 ULog gate;
11. [ ] implement and pass the 3 m/s MC pusher-feedback gate defined in
    `STANDARD_VTOL_NMPC_ROADMAP.md`;
12. [ ] add coordinated incremental PX4 transition tests.

The old open-loop identification launch must not be used as a prerequisite for
this controller.
