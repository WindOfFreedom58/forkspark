from Controllers import GeometricController

from cutils import CUtils
import numpy as np
import math

class StanleyControl(GeometricController):
    def __init__(self):
        super().__init__()

    def update_controls(self):
        x = self._current_x
        y = self._current_y
        yaw = self._current_yaw
        self.update_desired_speed()
        v_desired = self._desired_speed
        t = self._current_timestamp
        waypoints = self._waypoints
        throttle_output = 0
        steer_output = 0
        brake_output = 0

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            if not (self.stopping and self.STOPPED):
                # Find cross track error (assume point with closest distance)
                crosstrack_error = float("inf")
                crosstrack_vector = np.array([float("inf"), float("inf")])
                
                lookahead = self._base_lookahead_distance + v_desired * 0.3
                ce_idx = self.get_lookahead_index(lookahead)
                crosstrack_vector = np.array([waypoints.poses[ce_idx].pose.position.x - \
                                              x - lookahead * np.cos(yaw),
                                              waypoints.poses[ce_idx].pose.position.y - \
                                              y - lookahead * np.sin(yaw)])
                crosstrack_error = np.linalg.norm(crosstrack_vector)

                # set deadband to reduce oscillations
                # print("Crosstrack: " + str(crosstrack_error))
                if crosstrack_error < self.vars.cross_track_deadband:
                    crosstrack_error = 0.0

                # Compute the sign of the crosstrack error
                crosstrack_heading = np.arctan2(crosstrack_vector[1],
                                                crosstrack_vector[0])
                crosstrack_heading_error = crosstrack_heading - yaw
                crosstrack_heading_error = \
                    (crosstrack_heading_error + self._pi) % \
                    self._2pi - self._pi

                crosstrack_sign = np.sign(crosstrack_heading_error)

                # Compute heading relative to trajectory (heading error)
                # First ensure that we are not at the last index. If we are,
                # flip back to the first index (loop the waypoints)
                if ce_idx < len(waypoints.poses) - 1:
                    vect_wp0_to_wp1 = np.array(
                        [waypoints.poses[ce_idx + 1].pose.position.x - waypoints.poses[ce_idx].pose.position.x,
                         waypoints.poses[ce_idx + 1].pose.position.y - waypoints.poses[ce_idx].pose.position.y])
                    trajectory_heading = np.arctan2(vect_wp0_to_wp1[1],
                                                    vect_wp0_to_wp1[0])
                else:
                    vect_wp0_to_wp1 = np.array(
                        [waypoints.poses[0].pose.position.x - waypoints.poses[-1].pose.position.x,
                         waypoints.poses[0].pose.position.y - waypoints.poses[-1].pose.position.y])
                    trajectory_heading = np.arctan2(vect_wp0_to_wp1[1],
                                                    vect_wp0_to_wp1[0])

                heading_error = trajectory_heading - yaw
                heading_error = \
                    (heading_error + self._pi) % self._2pi - self._pi

                # Compute steering command based on error
                steer_output = heading_error + \
                               np.arctan(self.vars.kp_heading * \
                                         crosstrack_sign * \
                                         crosstrack_error / \
                                         (v_desired + self.vars.k_speed_crosstrack))

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_desired_speed(v_desired)  # in percent (0 to 1)
            # TODO: Tune this according to steering system, also tune the sign
            self.set_steer((steer_output/1.22)*100)  # in rad (-1.22 to 1.22) 
            self.set_brake(brake_output)  # in percent (0 to 1)

        if self._current_frame and not self._start_control_loop:
            self._start_control_loop = True
        self.vars.x_prev = x
        self.vars.y_prev = y
        self.vars.yaw_prev = yaw
        self.vars.v_error_prev = self.vars.v_error # TODO: not using this
        self.vars.t_prev = t

