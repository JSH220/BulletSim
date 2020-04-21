import math

import matplotlib.pyplot as plt
import numpy as np


class Config:

    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = 0.0  # [m/s]
        self.max_yawrate = 90.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.5  # [m/ss]
        self.max_dyawrate = math.pi # [rad/ss]
        self.v_reso = 0.1  # [m/s]
        self.yawrate_reso = 15.0 * math.pi / 180.0  # [rad/s]
        self.predict_time = 1.2  # [s]
        self.to_goal_cost_gain = 0.1
        self.speed_cost_gain = 1.5
        self.obstacle_cost_gain = 1.3

        self.robot_width = 0.4  # [m] for collision check
        self.robot_length = 0.7  # [m] for collision check

class DynamicWindowApproach(object):
    
    def __init__(self, goal, pos, angle, obst, vel_mod = 0, yaw_rate = 0):
        self._config = Config()
        self._goal = goal
        self._pos = pos
        self._angle = angle
        self._vel_mod = vel_mod
        self._yaw_rate = yaw_rate
        self._obst = obst
        self._time_step = 0.2
        self._state_vec = self._pos + [self._angle, self._vel_mod, self._yaw_rate]

    def update_state(self, goal, pos, angle, obst, vel_mod, yaw_rate):
        self._goal = goal
        self._pos = pos
        self._angle = angle
        self._vel_mod = vel_mod
        self._yaw_rate = yaw_rate
        self._obst = obst
        self._state_vec = self._pos + [self._angle, self._vel_mod, self._yaw_rate]

    def dwa_control(self):

        dw = self.calc_dynamic_window()
        u, trajectory = self.calc_control_and_trajectory(dw)

        return u, trajectory

    def calc_dynamic_window(self):

        # Dynamic window from robot specification
        Vs = [self._config.min_speed, self._config.max_speed,
            -self._config.max_yawrate, self._config.max_yawrate]

        # Dynamic window from motion model
        Vd = [self._vel_mod - self._config.max_accel * self._time_step,
            self._vel_mod + self._config.max_accel * self._time_step,
            self._yaw_rate - self._config.max_dyawrate * self._time_step,
            self._yaw_rate + self._config.max_dyawrate * self._time_step]

        #  [vmin, vmax, yaw_rate min, yaw_rate max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw

    def motion(self, x, u):

        x[2] += u[1] * self._time_step
        x[0] += u[0] * math.cos(x[2]) * self._time_step
        x[1] += u[0] * math.sin(x[2]) * self._time_step
        x[3] = u[0]
        x[4] = u[1]
        return x

    def predict_trajectory(self, v, y):

        x = np.array(self._state_vec[:])
        traj = np.array(x)
        time = 0
        while time <= self._config.predict_time:
            x = self.motion(x, [v, y])
            traj = np.vstack((traj, x))
            time += self._time_step
        return traj


    def calc_control_and_trajectory(self, dw):

        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([self._state_vec])
        to_goal_cost = 0
        speed_cost = 0
        ob_cost = 0
        # print("dw:", dw)
        for v in np.arange(dw[0], dw[1], self._config.v_reso):
            for y in np.arange(dw[2], dw[3], self._config.yawrate_reso):
                trajectory = self.predict_trajectory(v, y)
                # calc cost
                to_goal_cost = self._config.to_goal_cost_gain * self.calc_to_goal_cost(trajectory)
                speed_cost = self._config.speed_cost_gain * (self._config.max_speed - trajectory[-1, 3])
                ob_cost = self._config.obstacle_cost_gain * self.calc_obstacle_cost(trajectory)

                final_cost = to_goal_cost + speed_cost + ob_cost
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v / self._config.max_speed, y / self._config.max_yawrate]
                    best_trajectory = trajectory

        return best_u, best_trajectory


    def calc_obstacle_cost(self, trajectory):

        if not len(self._obst):
            return 0

        ox = self._obst[:, 0]
        oy = self._obst[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = self._obst[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= self._config.robot_length / 2
        right_check = local_ob[:, 1] <= self._config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -self._config.robot_length / 2
        left_check = local_ob[:, 1] >= -self._config.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                            np.logical_and(bottom_check, left_check))).any():
            return float("Inf")

        min_r = np.min(r)
        return 1.0 / min_r  # OK


    def calc_to_goal_cost(self, trajectory):

        dx = self._goal[0] - trajectory[-1, 0]
        dy = self._goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        return cost
