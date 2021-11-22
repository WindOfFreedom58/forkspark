"""
2D Controller Class to be used for the SPARK Waypoint Follower.
"""

from cutils import CUtils
import numpy as np
import math


class GeometricController:
    def __init__(self):
        self.vars = CUtils()

        self._base_lookahead_distance = 2.0
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0
        self._desired_speed = 0.0 # used for control
        self._current_frame = False
        self._current_timestamp = 0.0
        self._start_control_loop = False
        self._set_throttle = 0.0
        self._set_brake = 0.0
        self._set_steer = 0.0
        self._waypoints = None
        self._speeds = None
        self._conv_rad_to_steer = 180.0 / 70.0 / np.pi
        self._pi = np.pi
        self._2pi = 2.0 * np.pi

        self.vars.create_var('kp', 0.50)
        self.vars.create_var('ki', 0.30)
        self.vars.create_var('integrator_min', 0.0)
        self.vars.create_var('integrator_max', 10.0)
        self.vars.create_var('kd', 0.13)
        self.vars.create_var('kp_heading', 0.5)
        self.vars.create_var('k_speed_crosstrack', 0.1)
        self.vars.create_var('cross_track_deadband', 0.01)
        self.vars.create_var('x_prev', 0.0)
        self.vars.create_var('y_prev', 0.0)
        self.vars.create_var('yaw_prev', 0.0)
        self.vars.create_var('v_prev', 0.0)
        self.vars.create_var('t_prev', 0.0)
        self.vars.create_var('v_error', 0.0)
        self.vars.create_var('v_error_prev', 0.0)
        self.vars.create_var('v_error_integral', 0.0)

        self.desired_speed = 0.0 # updated from outside

        self.stopping = False
        self.STOPPED = False

        self.stop_time = None
        self.stopped_timestamp = None

    def update_values(self, x, y, yaw, timestamp, frame):
        self._current_x = x
        self._current_y = y
        self._current_yaw = yaw
        self._current_timestamp = timestamp
        self._current_frame = frame

    def get_lookahead_index(self, lookahead_distance):
        min_idx = 0
        min_dist = float("inf")
        for i in range(len(self._waypoints.poses)):
            dist = np.linalg.norm(np.array([
                self._waypoints.poses[i].pose.position.x - self._current_x,
                self._waypoints.poses[i].pose.position.y - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i

        total_dist = min_dist
        lookahead_idx = min_idx
        for i in range(min_idx + 1, len(self._waypoints.poses)):
            if total_dist >= lookahead_distance:
                break
            total_dist += np.linalg.norm(np.array([
                self._waypoints.poses[i].pose.position.x - self._waypoints.poses[i - 1].pose.position.x,
                self._waypoints.poses[i].pose.position.y - self._waypoints.poses[i - 1].pose.position.y]))
            lookahead_idx = i
        return lookahead_idx

    def update_desired_speed(self):
        min_idx = 0
        min_dist = float("inf")
        desired_speed = 0.0
        if self.stopping or self._waypoints is None:
            self._desired_speed = 0.0
            return
        for i in range(len(self._waypoints.poses)):
            dist = np.linalg.norm(np.array([
                self._waypoints.poses[i].pose.position.x - self._current_x,
                self._waypoints.poses[i].pose.position.y - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if self._speeds is not None:
            self._desired_speed = self._speeds[min_idx]
        else:
            self._desired_speed = self.desired_speed

    def update_waypoints(self, new_waypoints, speeds):
        self._waypoints = new_waypoints
        self._speeds = speeds

    def get_commands(self):
        return self.desired_speed, self._set_steer, 1.0 if self.desired_speed == 0 else 0.0

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer = np.fmax(np.fmin(input_steer, 100.0), -100.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def set_desired_speed(self, desired_speed):
        self.desired_speed = desired_speed
    
    def get_desired_speed(self):
        return self.desired_speed