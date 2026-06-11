# Standard VTOL 6DOF NMPC

Offline Python/CasADi experiment for a simplified standard VTOL quadplane
transition. This does not use ROS, PX4, Gazebo, or acados.

## Files

- `model.py`: 6DOF rigid-body dynamics with position, velocity, unit
  quaternion attitude, body rates, lift rotors, pusher thrust, moments, drag,
  and wing lift.
- `mpc_casadi.py`: multiple-shooting NMPC in CasADi `Opti` with Ipopt.
- `simulate.py`: closed-loop hover/low-speed to forward-flight transition.
- `plot_results.py`: plots the saved trajectory and controls.

## Run

On Ubuntu/Debian, do not install these packages into the system Python. Create
and use a virtual environment:

```bash
sudo apt install python3.12-venv
python3 -m venv --clear .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r standard_vtol_nmpc/requirements.txt
python -m standard_vtol_nmpc.simulate
python -m standard_vtol_nmpc.plot_results
```

The simulation writes:

```text
standard_vtol_nmpc/results/transition_6dof_results.npz
standard_vtol_nmpc/results/transition_6dof.png
```

## Model convention

State:

```text
[px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
```

Controls:

```text
[lift_thrust, pusher_thrust, roll_moment, pitch_moment, yaw_moment]
```

`pz` is altitude, positive up. Position and velocity are in the world frame.
The quaternion is `[qw, qx, qy, qz]` and maps body-frame vectors into the world
frame. Body rates `[wx, wy, wz]` are expressed in the body frame. The first
target is forward flight around `16 m/s` while holding `py = 0` and `pz = 0`.
