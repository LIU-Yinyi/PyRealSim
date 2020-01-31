from math import *
import numpy as np


def _relu(val, min_val=0.0):
    return val if val > min_val else 0.0


class Quadcopter:

    def __init__(self, freq=100.0):
        self.__force_hori_convert_coef = 0.4
        self.__force_vert_convert_coef = 0.126
        self.__aero_arm_half_length = 0.19
        self.__aero_inertia_constant = 0.56
        self.__aero_mass = 2.0
        self.__aero_land_height = 0.05
        self.__step_interval = 1.0 / freq
        self.__gravity = 9.8

        # might use @properties to optimze
        self.body_alpha = np.zeros(3, dtype=np.double)
        self.body_omega = np.zeros(3, dtype=np.double)
        self.body_theta = np.zeros(3, dtype=np.double)
        self.global_acc = np.zeros(3, dtype=np.double)
        self.global_vel = np.zeros(3, dtype=np.double)
        self.global_pos = np.zeros(3, dtype=np.double)
        self.global_pos[2] = self.__aero_land_height

        self.expect_pos = np.zeros(3, dtype=np.double)
        self.expect_yaw = np.zeros(1, dtype=np.double)
        self.expect_pos[2] = self.__aero_land_height

        self.func_ptr_map = {}

    def set_pos2d(self, pos2d):
        self.global_pos[0] = pos2d[0]
        self.global_pos[1] = pos2d[1]

    def get_pos2d(self):
        return self.global_pos[0:2]

    def get_land_heght(self):
        return self.__aero_land_height

    def model_evaluate(self, lf, lb, rb, rf):
        # assert non-negative
        lf = _relu(lf)
        lb = _relu(lb)
        rb = _relu(rb)
        rf = _relu(rf)

        # torque in FLU
        Fx = (lf + lb - rb - rf) * self.__force_hori_convert_coef
        Fy = (lb + rb - lf - rf) * self.__force_hori_convert_coef
        Fz = (lf + rb - rf - lb) * self.__force_vert_convert_coef
        Fh = (lf + lb + rb + rf) * self.__force_hori_convert_coef

        Fxyz = np.array([Fx, Fy, Fz], dtype=np.double)
        self.body_alpha = Fxyz * self.__aero_arm_half_length / self.__aero_inertia_constant

        if self.global_pos[2] <= self.__aero_land_height:
            self.body_alpha[0:2] = 0
            self.body_omega[0:2] = 0
            self.body_theta[0:2] = 0

        self.body_omega[0:2] += self.body_alpha[0:2] * self.__step_interval
        self.body_theta[0:2] += self.body_omega[0:2] * self.__step_interval

        self.global_acc[0] = self.__gravity * sin(self.body_theta[1]) * cos(self.body_theta[2])
        self.global_acc[1] = self.__gravity * sin(self.body_theta[0]) * sin(self.body_theta[2])

        self.global_vel[0:2] += self.global_acc[0:2] * self.__step_interval
        self.global_pos[0:2] += self.global_vel[0:2] * self.__step_interval

        self.global_pos[2] = Fh / self.__aero_mass - self.__gravity
        if self.global_pos[2] <= self.__aero_land_height and self.global_acc[2] < 0.0:
            self.global_acc[2] = 0.0

        self.global_vel[2] += self.global_acc[2] * self.__step_interval
        self.global_pos[2] += self.global_vel[2] * self.__step_interval

        if self.global_pos[2] <= self.__aero_land_height and self.global_vel[2] < 0.0:
            self.global_vel[2] = 0.0

        if self.global_pos[2] <= self.__aero_land_height:
            self.global_pos[2] = self.__aero_land_height

        self.body_omega[2] += self.body_alpha[2] * self.__step_interval
        self.body_theta[2] += self.body_omega[2] * self.__step_interval

    def update(self):
        lf = lb = rb = rf = 0.0
        if 'control' in self.func_ptr_map:
            lf, lb, rb, rf = self.func_ptr_map['control'](self.global_pos, self.global_vel, self.global_acc,
                                                          self.body_theta, self.body_omega, self.body_alpha,
                                                          self.expect_pos, self.expect_yaw)
        else:
            # default control function
            pass

        self.model_evaluate(lf, lb, rb, rf)
