# Standard VTOL Model Identification Plan

## Goal

Build an identified model of the PX4/Gazebo `standard_vtol` that is useful for
MPC design, transition tests, and feasibility analysis.

The first target is not a full 6DOF aerodynamic model. The practical target is a
grey-box model that describes the dynamics seen by the controller we actually
use in offboard mode. If the MPC sends PX4 rate/thrust setpoints, then the model
should identify the closed-loop response from those setpoints to vehicle motion.

## Why This Branch Exists

The previous full NMPC attempt often failed with `Maximum_Iterations_Exceeded`
or `Infeasible_Problem_Detected`, especially during transition. That suggests
one or more of these are wrong:

- the model does not match the simulated aircraft,
- the model is too complex for the current solver setup,
- the constraints/reference make the transition infeasible,
- the MPC outputs do not match the real PX4 lower-level interface.

This branch should reset the work around measured data.

## Identification Strategy

Use a staged grey-box approach:

1. Identify hover and vertical dynamics.
2. Identify pusher and forward-speed dynamics.
3. Identify pitch/body-rate response.
4. Combine those into a reduced longitudinal transition model.
5. Validate the model with data that was not used for fitting.
6. Use the identified model for MPC and feasibility maps.

## Initial Model Class

Start with a reduced longitudinal model:

```text
state x = [h, vx, vz, theta, q]
input u = [u_lift, u_pusher, q_cmd]
```

where:

- `h` is altitude in meters, positive up.
- `vx` is forward velocity along the initial transition direction.
- `vz` is vertical velocity, positive up.
- `theta` is pitch angle.
- `q` is pitch rate.
- `u_lift` is normalized lift thrust command.
- `u_pusher` is normalized pusher command.
- `q_cmd` is PX4 pitch-rate setpoint or the equivalent commanded pitch action.

Candidate continuous-time structure:

```text
h_dot     = vz
vx_dot    = a_p * u_pusher - d_x * vx * abs(vx) + a_theta * theta + b_x
vz_dot    = a_l * (u_lift - u_hover) - d_z * vz + c_theta * vx * theta + b_z
theta_dot = q
q_dot     = (q_cmd - q) / tau_q
```

This is intentionally simple. It captures the minimum pieces needed for hover
and transition:

- lift authority,
- pusher acceleration,
- vertical damping,
- forward drag,
- pitch-rate response,
- coupling between pitch/forward speed/vertical motion.

If this is too simple, add terms only when residuals show a clear need.

## Data Collection Experiments

Record each experiment as a separate rosbag with a short metadata file that
contains parameters, command amplitudes, initial conditions, and notes.

### 1. Hover Thrust Calibration

Purpose:

- estimate `u_hover`,
- estimate lift-to-vertical-acceleration gain near hover.

Maneuver:

- start in stable hover at 2 m to 5 m altitude,
- command fixed lift values around hover,
- use small steps, for example `0.505`, `0.512`, `0.518`, `0.524`, `0.530`,
- keep rates near zero.

Expected output:

- refined hover thrust,
- vertical acceleration vs lift curve.

### 2. Vertical Lift Excitation

Purpose:

- estimate vertical damping and lift dynamics.

Maneuver:

- use lift sine/chirp/PRBS around hover,
- keep pusher zero,
- keep roll/pitch/yaw rates near zero,
- keep altitude range safe.

Expected output:

- parameters `a_l`, `d_z`, maybe lift delay/time constant if visible.

### 3. Pusher Excitation

Purpose:

- estimate forward acceleration and drag.

Maneuver:

- start at safe altitude,
- hold lift near hover or use PX4 altitude hold baseline,
- apply pusher steps/sine/PRBS,
- keep pitch-rate command small at first.

Expected output:

- parameters `a_p`, `d_x`,
- check whether pusher command mapping is linear.

### 4. Pitch-Rate Response

Purpose:

- estimate how PX4 lower-level control tracks rate commands.

Maneuver:

- keep lift near hover,
- pusher zero or small,
- apply small pitch-rate steps/chirp,
- record actual pitch rate and pitch angle.

Expected output:

- `tau_q`,
- rate limits,
- observed delay.

### 5. Combined Transition-Like Excitation

Purpose:

- validate coupling terms before MPC transition.

Maneuver:

- start from hover,
- slowly ramp pusher,
- apply bounded pitch-rate profile,
- keep lift in a conservative safe band,
- do not use aggressive constraints yet.

Expected output:

- validation data for the combined model,
- evidence of which terms are missing.

## ROS Topics To Record

Minimum:

```bash
ros2 bag record -o bags/standard_vtol_id_hover_01 \
  /fmu/out/vehicle_odometry \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_attitude \
  /fmu/out/vehicle_angular_velocity \
  /fmu/out/vehicle_status \
  /fmu/out/vtol_vehicle_status \
  /fmu/in/offboard_control_mode \
  /fmu/in/vehicle_rates_setpoint
```

Add when available:

```bash
ros2 bag record -o bags/standard_vtol_id_transition_01 \
  /fmu/out/vehicle_odometry \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_attitude \
  /fmu/out/vehicle_angular_velocity \
  /fmu/out/vehicle_status \
  /fmu/out/vtol_vehicle_status \
  /fmu/out/airspeed_validated_v1 \
  /fmu/in/offboard_control_mode \
  /fmu/in/vehicle_rates_setpoint
```

If direct actuator or pusher-specific commands are used later, record those
topics too.

## Future Tooling

Planned scripts:

```text
tools/standard_vtol_id/excitation_node.py
tools/standard_vtol_id/extract_bag.py
tools/standard_vtol_id/fit_model.py
tools/standard_vtol_id/validate_model.py
tools/standard_vtol_id/feasibility_map.py
```

The excitation node should support:

```text
lift_sweep
lift_prbs
pusher_steps
pusher_prbs
pitch_rate_steps
combined_transition
```

The fitting pipeline should produce:

```text
models/standard_vtol_identified_v1.yaml
results/standard_vtol_id/<date>/fit_report.md
results/standard_vtol_id/<date>/validation_plots/
```

## Data Processing

For every bag:

1. Convert PX4 NED data into the model frame.
   - altitude `h = -z_ned`
   - vertical velocity `vz = -vz_ned`
   - forward velocity should be projected onto the selected transition heading
2. Synchronize input and state topics onto one time grid.
3. Remove takeoff/landing and unsafe transients from the fitting window.
4. Filter noisy velocity/acceleration estimates.
5. Compute derivatives using filtered finite differences or Savitzky-Golay.
6. Split data by maneuver into train and validation sets.

## Parameter Estimation

Use nonlinear least squares first:

```text
minimize sum ||x_measured[k+1] - x_model[k+1]||^2
```

Fit in stages:

1. `u_hover`, `a_l`, `d_z` from hover/vertical tests.
2. `a_p`, `d_x` from pusher tests.
3. `tau_q` from pitch-rate tests.
4. coupling terms from combined tests.

Then validate with rollout simulation, not only one-step prediction.

## Validation Criteria

A model is acceptable for first MPC experiments if validation data shows:

- altitude rollout does not drift badly over 5 s to 10 s windows,
- forward-speed prediction captures acceleration trend,
- pitch-rate response matches delay and bandwidth,
- residuals are not strongly biased in one flight condition,
- the model predicts transition-like data better than a constant-velocity or
  constant-acceleration baseline.

Exact numerical thresholds should be set after seeing the first datasets.

## Feasibility Analysis

Once the identified discrete model exists, run constrained simulations and MPC
feasibility maps:

- minimum altitude needed to reach target airspeed,
- transition time vs pusher authority,
- altitude loss vs initial speed,
- feasible/infeasible regions under thrust and rate limits,
- sensitivity to mass, hover thrust, drag, and solver horizon.

This is where the scientific contribution can become clearer: the identified
model gives a measured basis for explaining which transitions are physically
and numerically feasible, instead of only tuning NMPC until it works once.

## First Concrete Milestone

The first milestone is small:

1. Create a safe excitation node for lift and pusher steps.
2. Record three hover/lift bags and three pusher bags.
3. Fit the first `[h, vx, vz]` model.
4. Produce one validation plot and one parameter YAML file.

Only after that should we return to full NMPC transition.
