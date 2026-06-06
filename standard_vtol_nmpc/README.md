# Standard VTOL NMPC phase 1

Offline Python/CasADi experiment for a simplified standard VTOL quadplane
transition. This does not use ROS, PX4, Gazebo, or acados.

## Files

- `model.py`: longitudinal 2D dynamics with lift rotors, pusher thrust, pitch
  moment, drag, and wing lift.
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
standard_vtol_nmpc/results/transition_results.npz
standard_vtol_nmpc/results/transition.png
```

## Model convention

State:

```text
[px, pz, vx, vz, theta, q]
```

Controls:

```text
[lift_thrust, pusher_thrust, pitch_moment]
```

`pz` is altitude, positive up. The first target is forward flight around
`16 m/s` while holding altitude near zero.
