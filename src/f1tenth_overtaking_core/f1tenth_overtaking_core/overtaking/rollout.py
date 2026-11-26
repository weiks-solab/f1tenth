# SPDX-License-Identifier: Apache-2.0
# Model integration and rollout adapted from f1tenth_benchmarks at 008907f3826ddf486f88dcd6079aa6d4c240084e.
"""Single-track rollout independent of the simulation environment."""

import numpy as np

from .dynamic_models import pid, vehicle_dynamics_st


class SingleTrackRolloutState:
    """Private copy of the measured or replay state and steering delay."""

    def __init__(self, params, timestep, state, steer_buffer):
        self.params = params
        self.time_step = timestep
        self.state = np.array(state, dtype=float, copy=True)
        self.steer_buffer = np.array(steer_buffer, dtype=float, copy=True)
        self.steer_buffer_size = 2
        if (self.state.shape != (7,) or self.steer_buffer.ndim != 1
                or len(self.steer_buffer) > self.steer_buffer_size
                or not np.isfinite(self.state).all() or not np.isfinite(self.steer_buffer).all()):
            raise ValueError('invalid single-track rollout state')

    def update_pose(self, raw_steer, vel):
        """Advance one model step under the requested steering and speed."""
        # state is [x, y, steer_angle, vel, yaw_angle, yaw_rate, slip_angle]

        # steering delay
        steer = 0.
        if self.steer_buffer.shape[0] < self.steer_buffer_size:
            steer = 0.
            self.steer_buffer = np.append(raw_steer, self.steer_buffer)
        else:
            steer = self.steer_buffer[-1]
            self.steer_buffer = self.steer_buffer[:-1]
            self.steer_buffer = np.append(raw_steer, self.steer_buffer)

        # steering angle velocity input to steering velocity acceleration input
        accl, sv = pid(vel, steer, self.state[3], self.state[2], self.params['sv_max'], self.params['a_max'], self.params['v_max'], self.params['v_min'])

        f = vehicle_dynamics_st(
            self.state,
            np.array([sv, accl]),
            self.params['mu'],
            self.params['C_Sf'],
            self.params['C_Sr'],
            self.params['lf'],
            self.params['lr'],
            self.params['h'],
            self.params['m'],
            self.params['I'],
            self.params['s_min'],
            self.params['s_max'],
            self.params['sv_min'],
            self.params['sv_max'],
            self.params['v_switch'],
            self.params['a_max'],
            self.params['v_min'],
            self.params['v_max'])
        self.state = self.state + self.time_step * f

        # bound yaw angle
        if self.state[4] > 2*np.pi:
            self.state[4] = self.state[4] - 2*np.pi
        elif self.state[4] < 0:
            self.state[4] = self.state[4] + 2*np.pi

        return self.state


class SingleTrackRollout:
    """Run the existing model and collision checks from an explicit snapshot.

    The shipped parameter set is for simulation parity. Vehicle parameters,
    steering history, yaw rate and slip state require hardware validation.
    """

    def __init__(self, controller, body, occupancy, *, params, timestep, steer_ticks,
                 pair_clearance, wall_clearance, physics_tick, state, steer_buffer):
        self.controller = controller
        self.body = body
        self.occupancy = occupancy
        self.params = params
        self.timestep = timestep
        self.steer_ticks = steer_ticks
        self.pair_margin = pair_clearance + .04
        self.wall_margin = wall_clearance + .02
        self.physics_tick = physics_tick
        self.state = state
        self.steer_buffer = steer_buffer
        self.sample_steps = max(1, round(.05 / timestep))

    def __call__(self, pp, speed, lead_centres, lead_rate, times, path_s=None, *, held_steering):
        car = SingleTrackRolloutState(self.params, self.timestep, self.state, self.steer_buffer)
        first_index = pp.nearest(car.state[:2])[2:]
        steer = held_steering
        minimum = float('inf')
        first_issue = None
        for i, t in enumerate(times):
            if i:
                try:
                    for substep in range(self.sample_steps):
                        tick = (i - 1) * self.sample_steps + substep
                        if (self.physics_tick + tick) % self.steer_ticks == 0:
                            action, _ = pp.plan(car.state[[0, 1, 4]], self.controller.lookahead(car.state[3]))
                            steer = action[0]
                        car.update_pose(steer, speed)
                except ValueError:
                    if first_issue is None:
                        first_issue = ('lookahead_unavailable', float(t))
                    break
            centres = self.body.centres(car.state[[0, 1, 4]])
            clearance = float(np.linalg.norm(centres[:, None] - lead_centres[i][None, :], axis=2).min()
                              - 2 * self.body.radius)
            minimum = min(minimum, clearance)
            if first_issue is None:
                if clearance <= self.pair_margin:
                    first_issue = ('predicted_pair_clearance', float(t))
                elif self.occupancy.intersects(centres, self.body.radius + self.wall_margin):
                    first_issue = ('predicted_wall_clearance', float(t))
        final_index = pp.nearest(car.state[:2])[2:]
        if path_s is None:
            s_per_segment = self.controller.config.lookahead_m / (len(pp.path) - 1)
            advance = (sum(final_index) - sum(first_index)) * s_per_segment
        else:
            def at(projection):
                fraction, index = projection
                return path_s[index] + fraction*(path_s[index+1]-path_s[index])
            advance = at(final_index) - at(first_index)
        relative_progress = advance - lead_rate * t
        return dict(safe=first_issue is None, reason=first_issue[0] if first_issue else None,
                    first_rejection_s=first_issue[1] if first_issue else None, min_pair_clearance_m=minimum,
                    relative_s_progress_m=relative_progress)
