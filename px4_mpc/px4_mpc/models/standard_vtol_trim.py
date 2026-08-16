"""Steady longitudinal trim corridor for the reduced Standard VTOL model."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from px4_mpc.models.standard_vtol_gz_model import StandardVtolGazeboModel


@dataclass(frozen=True)
class TransitionTrimPoint:
    """One steady point of the cascaded rate-controlled transition model."""

    airspeed: float
    pitch: float
    collective_lift: float
    pusher: float
    elevator: float
    acceleration_world: tuple[float, float, float]
    required_pitch_acceleration: float


class StandardVtolTrimSolver:
    """Generate force-balanced trims while PX4 closes the body-rate loop."""

    pitch_min = math.radians(-20.0)
    pitch_max = math.radians(15.0)
    lift_min = 0.0
    lift_max = 0.65
    pusher_min = 0.0
    pusher_max = 0.60

    def __init__(self, plant: StandardVtolGazeboModel | None = None) -> None:
        self.plant = StandardVtolGazeboModel() if plant is None else plant

    @staticmethod
    def _pitch_quaternion(pitch: float) -> np.ndarray:
        return np.array([math.cos(0.5 * pitch), 0.0, math.sin(0.5 * pitch), 0.0])

    def derivative(
        self,
        airspeed: float,
        pitch: float,
        collective_lift: float,
        pusher: float,
        elevator: float = 0.0,
    ) -> np.ndarray:
        """Evaluate the full plant at a candidate steady-flight point."""
        state = np.zeros(18)
        state[3] = airspeed
        state[6:10] = self._pitch_quaternion(pitch)
        state[13:17] = [
            motor.target_speed(collective_lift)
            for motor in self.plant.motors[:4]
        ]
        state[17] = self.plant.motors[4].target_speed(pusher)
        control = np.array([collective_lift] * 4 + [pusher, 0.0, 0.0, elevator])
        return self.plant.derivative(state, control)

    def _residual(
        self, airspeed: float, values: np.ndarray, include_pitch_moment: bool
    ) -> np.ndarray:
        derivative = self.derivative(
            airspeed, values[0], values[1], values[2], values[3]
        )
        indices = [3, 5, 11] if include_pitch_moment else [3, 5]
        return derivative[indices]

    def _solve_commands(
        self,
        airspeed: float,
        pitch: float,
        initial_commands: np.ndarray,
        include_pitch_moment: bool,
    ) -> tuple[np.ndarray, float]:
        values = np.array([pitch, *initial_commands], dtype=float)
        lower = np.array([pitch, self.lift_min, self.pusher_min, -0.78])
        upper = np.array([pitch, self.lift_max, self.pusher_max, 0.78])
        command_indices = (1, 2, 3) if include_pitch_moment else (1, 2)

        for _ in range(30):
            residual = self._residual(airspeed, values, include_pitch_moment)
            if np.linalg.norm(residual, ord=np.inf) < 1.0e-8:
                break
            jacobian = np.zeros((len(residual), len(command_indices)))
            for column, value_index in enumerate(command_indices):
                step = 1.0e-5
                shifted = values.copy()
                shifted[value_index] += step
                jacobian[:, column] = (
                    self._residual(airspeed, shifted, include_pitch_moment) - residual
                ) / step
            command_step = np.linalg.lstsq(jacobian, -residual, rcond=None)[0]
            base_norm = np.linalg.norm(residual)
            scale = 1.0
            while scale >= 1.0e-5:
                candidate = values.copy()
                candidate[list(command_indices)] += scale * command_step
                candidate = np.clip(candidate, lower, upper)
                if np.linalg.norm(
                    self._residual(airspeed, candidate, include_pitch_moment)
                ) < base_norm:
                    values = candidate
                    break
                scale *= 0.5
            else:
                break

        error = float(
            np.linalg.norm(
                self._residual(airspeed, values, include_pitch_moment), ord=np.inf
            )
        )
        return values[1:4], error

    def _zero_lift_candidate(self, airspeed: float) -> np.ndarray | None:
        """Solve the wing-borne trim branch with lift rotors fully unloaded."""
        if airspeed < 8.0:
            return None
        nominal = self.plant.nominal_level_flight_trim(airspeed)
        values = np.array(
            [math.radians(-5.0), nominal.pusher_command, math.radians(20.0)]
        )
        lower = np.array([self.pitch_min, self.pusher_min, -0.78])
        upper = np.array([self.pitch_max, self.pusher_max, 0.78])

        def residual(candidate: np.ndarray) -> np.ndarray:
            derivative = self.derivative(
                airspeed, candidate[0], 0.0, candidate[1], candidate[2]
            )
            return derivative[[3, 5, 11]]

        for _ in range(40):
            current = residual(values)
            if np.linalg.norm(current, ord=np.inf) < 1.0e-8:
                break
            jacobian = np.column_stack(
                [
                    (residual(values + np.eye(3)[index] * 1.0e-5) - current)
                    / 1.0e-5
                    for index in range(3)
                ]
            )
            step = np.linalg.lstsq(jacobian, -current, rcond=None)[0]
            base_norm = np.linalg.norm(current)
            scale = 1.0
            while scale >= 1.0e-5:
                candidate = np.clip(values + scale * step, lower, upper)
                if np.linalg.norm(residual(candidate)) < base_norm:
                    values = candidate
                    break
                scale *= 0.5
            else:
                return None

        if np.linalg.norm(residual(values), ord=np.inf) > 1.0e-5:
            return None
        return np.array([values[0], 0.0, values[1], values[2]])

    @staticmethod
    def _stage_cost(pitch: float, lift: float, pusher: float, elevator: float) -> float:
        """Prefer unloading lift rotors without excessive pitch or pusher."""
        pitch_scale = math.radians(20.0)
        return (
            30.0 * lift**2
            + 0.25 * pusher**2
            + 0.08 * (pitch / pitch_scale) ** 2
            + 0.02 * (elevator / 0.78) ** 2
        )

    @staticmethod
    def _transition_cost(previous: np.ndarray, current: np.ndarray) -> float:
        pitch_step = (current[0] - previous[0]) / math.radians(5.0)
        lift_step = (current[1] - previous[1]) / 0.10
        pusher_step = (current[2] - previous[2]) / 0.10
        elevator_step = (current[3] - previous[3]) / math.radians(10.0)
        return (
            0.15 * pitch_step**2
            + 0.02 * lift_step**2
            + 0.04 * pusher_step**2
            + 0.02 * elevator_step**2
        )

    def _candidates(self, airspeed: float, pitch_step_degrees: float) -> np.ndarray:
        if airspeed <= 1.0e-9:
            derivative = self.derivative(0.0, 0.0, self.plant.hover_command, 0.0)
            if np.linalg.norm(derivative[[3, 5]], ord=np.inf) > 1.0e-6:
                raise RuntimeError("plant hover point is not force balanced")
            return np.array([[0.0, self.plant.hover_command, 0.0, 0.0]])

        pitch_values = np.deg2rad(
            np.arange(
                math.degrees(self.pitch_min),
                math.degrees(self.pitch_max) + 0.5 * pitch_step_degrees,
                pitch_step_degrees,
            )
        )
        nominal = self.plant.nominal_level_flight_trim(airspeed)
        initial = np.array(
            [nominal.collective_lift_command, nominal.pusher_command, 0.0]
        )
        include_pitch_moment = airspeed >= 5.0
        candidates = []
        for pitch in pitch_values:
            commands, error = self._solve_commands(
                airspeed, pitch, initial, include_pitch_moment
            )
            if error <= 1.0e-5:
                candidates.append([pitch, commands[0], commands[1], commands[2]])
                initial = commands
        zero_lift = self._zero_lift_candidate(airspeed)
        if zero_lift is not None:
            candidates.append(zero_lift.tolist())
        if not candidates:
            raise RuntimeError(f"no force-balanced trim found at {airspeed:.2f} m/s")
        return np.asarray(candidates)

    def corridor(
        self,
        airspeeds: np.ndarray,
        pitch_step_degrees: float = 0.25,
    ) -> list[TransitionTrimPoint]:
        """Find a globally smooth minimum-cost sequence of trim points."""
        airspeeds = np.asarray(airspeeds, dtype=float)
        if airspeeds.ndim != 1 or len(airspeeds) == 0:
            raise ValueError("airspeeds must be a nonempty one-dimensional array")
        if np.any(airspeeds < 0.0) or np.any(np.diff(airspeeds) <= 0.0):
            raise ValueError("airspeeds must be nonnegative and strictly increasing")

        layers = [self._candidates(speed, pitch_step_degrees) for speed in airspeeds]
        costs = np.array(
            [self._stage_cost(*candidate) for candidate in layers[0]]
        )
        parents: list[np.ndarray] = []
        for layer in layers[1:]:
            next_costs = np.full(len(layer), np.inf)
            parent = np.zeros(len(layer), dtype=int)
            for index, candidate in enumerate(layer):
                options = costs + np.array(
                    [self._transition_cost(previous, candidate) for previous in layers[len(parents)]]
                )
                # A hover-to-forward transition must not ask lift rotors to
                # spool back up as airspeed increases. Small numerical noise
                # is tolerated by the pitch grid, but the schedule is
                # otherwise monotonically unloading.
                previous_layer = layers[len(parents)]
                options = np.where(
                    previous_layer[:, 1] + 1.0e-6 >= candidate[1],
                    options,
                    np.inf,
                )
                if not np.any(np.isfinite(options)):
                    continue
                parent[index] = int(np.argmin(options))
                next_costs[index] = options[parent[index]] + self._stage_cost(*candidate)
            if not np.any(np.isfinite(next_costs)):
                raise RuntimeError("no monotonically unloading trim path exists")
            parents.append(parent)
            costs = next_costs

        indices = [int(np.argmin(costs))]
        for parent in reversed(parents):
            indices.append(int(parent[indices[-1]]))
        indices.reverse()

        result = []
        for airspeed, layer, index in zip(airspeeds, layers, indices):
            pitch, lift, pusher, elevator = layer[index]
            derivative = self.derivative(airspeed, pitch, lift, pusher, elevator)
            result.append(
                TransitionTrimPoint(
                    airspeed=float(airspeed),
                    pitch=float(pitch),
                    collective_lift=float(lift),
                    pusher=float(pusher),
                    elevator=float(elevator),
                    acceleration_world=tuple(derivative[3:6]),
                    required_pitch_acceleration=float(derivative[11]),
                )
            )
        return result
