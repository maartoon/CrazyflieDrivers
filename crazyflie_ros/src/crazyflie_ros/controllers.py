# ==================================================================================
#
#       File: controllers.py
#       Description: This file contains the template for the drone's flight
#                    controller. You will implement the control logic
#                    within the `solve` method of the DroneController class.
#                    The controller is responsible for calculating the required
#                    accelerations and yaw rate to follow a given trajectory.
#
# ==================================================================================



import casadi as ca
import numpy as np
import time
import math


class DroneController:
    def __init__(self, dt=0.01, vmax=1, amax=7.0):
        """
        Initializes the controller. You can add any parameters
        or initializations that are needed here (e.g., PID gains, MPC parameters).

        NOTE : Build Your controller respecting the velocity and acceleration constraints of crazyflie.

        Args:
            dt (float): The time step (sampling time) of the controller, in seconds.
            vmax (float): Maximum crazyflies velocity in m/s.
            amax (float): Maximum crazyflies acceleration in m/s^2. 

        """

        self.dt = dt  #100hz
        self.vmax = vmax
        self.amax = amax


        # PID gains for x, y, z axes control.
        self.KP = np.array([7.9, 7.9, 10.0])  # position gains
        self.KV = np.array([4.0, 4.0, 8.0])    # velocity gains (damping)
        self.KI = np.array([0.05, 0.05, 0.004])
        # YAW gains
        self.YAW_KP = 2.0
        self.YAW_KD = 0.8

        self.INTEGRAL_LIMIT = np.array([2.0, 2.0, 2.0])
        self._integral_error = np.zeros(3)
        self._prev_yaw_error = 0.0
    def reset_integrator(self):
        """Expose a helper to reset PID integrator memory between simulations."""
        self._integral_error = np.zeros(3)
        self._prev_yaw_error = 0.0

    def _wrap_angle(angle):
        """Wrap any angle to [-pi, pi] for consistent yaw error calculations."""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    def solve(self, current_state, reference_states):
        """
        Calculates the required control commands (accelerations and yaw rate)
        to follow the reference trajectory.

        This is the primary function that students need to implement.

        Args:
            current_state (np.ndarray): A 1D NumPy array containing the drone's
                current state, with the following format:
                [x, y, z, vx, vy, vz, yaw]
                - x, y, z: current position in the world frame (meters)
                - vx, vy, vz: current velocity in the world frame (m/s)
                - yaw: current yaw angle in radians

            reference_states (list of np.ndarray): A list where each element is a
                1D NumPy array representing a future reference state. The format
                of each array is the same as `current_state`:
                [x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref, yaw_ref]
                - For a simple PID controller, you might only use the first
                  element of this list (reference_states[0]).
                - For an MPC controller, you can use the entire list as your
                  prediction horizon. If you want to increase the horizon, update your trajectory function to publish more points.

        Returns:
            tuple: A tuple containing the calculated control commands:
                (ax_cmd, ay_cmd, az_cmd, yaw_rate_cmd)
                - ax_cmd, ay_cmd, az_cmd: desired linear accelerations in the
                  world frame (m/s^2).
                - yaw_rate_cmd: desired yaw rate (rad/s).
        """

        #TODO: Implement your control logic here.
        # You can use libraries like CasADi for optimization if needed.
        # Make sure to respect the velocity and acceleration constraints of crazyflie.
        state = np.asarray(current_state, dtype=float).reshape(7)
        reference = np.asarray(reference_states, dtype=float).reshape(7)

        p   = state[0:3]
        v   = state[3:6]
        yaw = state[6]

        p_d   = reference[0:3]
        v_d   = reference[3:6]
        yaw_d = reference[6]

        e_p = p_d - p
        e_v = v_d - v

        self._integral_error += e_p * self.dt
        self._integral_error = np.clip(_integral_error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)

        # altitude-priority blending
        ez = e_p[2]
        ez_max = 0.5  # meters. smaller => stronger "z first"
        alpha = np.clip(1.0 - abs(ez)/ez_max, 0.0, 1.0)

        accel_cmd = self.KP * e_p + self.KI * _integral_error + self.KV * e_v
        accel_cmd[0:2] *= alpha
        accel_cmd = np.clip(accel_cmd, -self.amax, self.amax)

        # Convert world accelerations into body-frame components for dynamics integration.
        c, s = np.cos(yaw), np.sin(yaw)
        ax_body = accel_cmd[0] * c + accel_cmd[1] * s
        ay_body = -accel_cmd[0] * s + accel_cmd[1] * c
        accel_body = np.array([ax_body, ay_body, accel_cmd[2]])

        e_yaw = _wrap_angle(yaw_d - yaw)
        e_yaw_rate = (e_yaw - self._prev_yaw_error)/self.dt
        yaw_rate_cmd = self.YAW_KP*e_yaw + self.YAW_KD*e_yaw_rate
        self._prev_yaw_error = e_yaw

        control = np.hstack((accel_body, yaw_rate_cmd))
        control = np.clip(control, -2.0, 2.0)
        return tuple(control.tolist())



