from math import *
from .display import *
import numpy as np

'''
>>>> COORDINATE SYSTEM <<<<
assume: body head face North (FLU)
when roll > 0: Fly to East
when pitch > 0: Fly to South
when yaw > 0: From North to West
'''


def _bound(val, min_val, max_val):
    assert(min_val < max_val)
    val = min_val if val < min_val else val
    val = max_val if val > max_val else val
    return val


def _bound(val, bd):
    assert(bd > 0.0)
    val = -bd if val < -bd else val
    val = bd if val > bd else val
    return val


def _relu(val, min_val=0.0):
    return val if val > min_val else 0.0


def _deg2rad(deg):
    return deg * pi / 180.0


def _rad2deg(rad):
    return rad * 180.0 / pi


def cosd(deg):
    return cos(_deg2rad(deg))


def sind(deg):
    return sin(_deg2rad(deg))


def rot2d(vec, deg):
    # ### vec -> ret : +deg for Right-Land Coordinate (F: +x, L: +y) ### #
    assert(type(vec) == np.ndarray and vec.shape == (2,))
    ret = np.array([0.0, 0.0], dtype=np.double)
    ret[0] = vec[0] * cosd(deg) + vec[1] * sind(deg)
    ret[1] = -vec[0] * sind(deg) + vec[1] * cosd(deg)
    return ret


def rot3dx(vec, roll):
    Rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, cosd(roll), -sind(roll)],
                   [0.0, sind(roll), cosd(roll)]],
                  dtype=np.double)
    return np.matmul(vec, Rx)


def rot3dy(vec, pitch):
    Ry = np.array([[cosd(pitch), 0.0, -sind(pitch)],
                   [0.0, 1.0, 0.0],
                   [sind(pitch), 0.0, cosd(pitch)]],
                  dtype=np.double)
    return np.matmul(vec, Ry)


def rot3dz(vec, yaw):
    Rz = np.array([[cosd(yaw), -sind(yaw), 0.0],
                   [sind(yaw), cosd(yaw), 0.0],
                   [0.0, 0.0, 1.0]],
                  dtype=np.double)
    return np.matmul(vec, Rz)


def rot3d(vec, yaw, pitch, roll):
    vec = rot3dz(vec, yaw)
    vec = rot3dy(vec, pitch)
    vec = rot3dx(vec, roll)
    return vec


def derot3d(vec, yaw, pitch, roll):
    vec = rot3dx(vec, -roll)
    vec = rot3dy(vec, -pitch)
    vec = rot3dz(vec, -yaw)
    return vec


class Quadcopter:

    def __init__(self, freq=100.0):
        self.__force_hori_convert_coef = 0.4
        self.__force_vert_convert_coef = 0.6
        self.__aero_arm_half_length = 0.19
        self.__aero_inertia_constant = 0.01
        self.__aero_mass = 1.8
        self.__aero_land_height = 0.10
        self.__step_interval = float(1.0 / freq)
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
        print(GREEN + "[Quadcopter] Physics init success.")

    def reg_func(self, name, func):
        self.func_ptr_map[name] = func

    def get_dt(self):
        return self.__step_interval

    def set_pos2d(self, pos2d):
        self.global_pos[0] = pos2d[0]
        self.global_pos[1] = pos2d[1]

    def get_pos2d(self):
        return self.global_pos[0:2]

    def get_land_heght(self):
        return self.__aero_land_height

    def model_evaluate(self, lf, lb, rb, rf):
        # ## assert non-negative ## #
        # lf = _relu(lf)
        # lb = _relu(lb)
        # rb = _relu(rb)
        # rf = _relu(rf)

        # torque in FLU
        Fx = (lf + lb - rb - rf) * self.__force_hori_convert_coef
        Fy = (lb + rb - lf - rf) * self.__force_hori_convert_coef
        Fz = (lf + rb - rf - lb) * self.__force_vert_convert_coef
        Fh = (lf + lb + rb + rf) * self.__force_hori_convert_coef * cosd(self.body_theta[0]) * cosd(self.body_theta[1])

        Fxyz = np.array([Fx, Fy, Fz], dtype=np.double)
        self.body_alpha = Fxyz * self.__aero_arm_half_length / self.__aero_inertia_constant

        if self.global_pos[2] <= self.__aero_land_height:
            self.body_alpha[0:2] = 0.0
            self.body_omega[0:2] = 0.0
            self.body_theta[0:2] = 0.0

        self.body_omega[0:2] += self.body_alpha[0:2] * self.__step_interval
        self.body_theta[0:2] += self.body_omega[0:2] * self.__step_interval

        body_acc = np.array([0.0, 0.0], dtype=np.double)
        body_acc[0] = self.__gravity * sind(-self.body_theta[1])
        body_acc[1] = self.__gravity * sind(-self.body_theta[0])
        self.global_acc[0:2] = rot2d(body_acc, -self.body_theta[2])

        self.global_vel[0:2] += self.global_acc[0:2] * self.__step_interval
        self.global_pos[0:2] += self.global_vel[0:2] * self.__step_interval

        self.global_acc[2] = Fh / self.__aero_mass - self.__gravity
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

        yaw_assert = fmod(self.body_theta[2] + 180.0, 360.0)
        if yaw_assert < 0.0:
            yaw_assert += 360.0
        yaw_assert = fmod(yaw_assert, 360.0) - 180.0
        self.body_theta[2] = yaw_assert

    def update(self):
        # if _dt > 0.0:
        #    self.__step_interval = _dt

        lf = lb = rb = rf = 0.0
        if 'control' in self.func_ptr_map:
            lf, lb, rb, rf = self.func_ptr_map['control'](self.global_pos, self.global_vel, self.global_acc,
                                                          self.body_theta, self.body_omega, self.body_alpha,
                                                          self.expect_pos, self.expect_yaw, self.__step_interval)
        else:
            # ### default control function ### #
            pos = self.global_pos.copy()
            vel = self.global_vel.copy()
            acc = self.global_acc.copy()
            theta = self.body_theta.copy()
            omega = self.body_omega.copy()
            alpha = self.body_alpha.copy()
            exp_pos = self.expect_pos.copy()
            exp_yaw = self.expect_yaw.copy()
            dt = self.__step_interval

            # ## parameters of pid ## #
            euler_hori_p = 0.32  # work
            euler_hori_i = 1.1  # work
            euler_vert_p = 0.075  # work
            euler_vert_i = 0.11  # work
            euler_hori_max_angle = 45.0  # work
            euler_vert_max_angle = 60.0  # work

            pos_hori_p = 22
            pos_hori_i = 14
            pos_hori_d = 10
            pos_hori_max = 10.0
            pos_vert_p = 7.5  # work
            pos_vert_i = 14  # work

            # ## horizontal control - outer loop## #
            err_p = exp_pos[0:2] - pos[0:2]
            err_p_norm = np.linalg.norm(err_p)
            if err_p_norm > pos_hori_max:
                err_p = err_p / err_p_norm * pos_hori_max
            err_vp = -vel[0:2]
            out_p = err_vp * pos_hori_p + err_p * pos_hori_i - acc[0:2] * pos_hori_d
            # need rotate matrix
            exp_rollpitch = rot2d(out_p, theta[2])
            exp_roll = _bound(-exp_rollpitch[1], euler_hori_max_angle)
            exp_pitch = _bound(-exp_rollpitch[0], euler_hori_max_angle)

            # ## horizontal control - inter loop ## #
            err_roll = exp_roll - theta[0]
            out_roll = - omega[0] * euler_hori_p + err_roll * euler_hori_i

            err_pitch = exp_pitch - theta[1]
            out_pitch = - omega[1] * euler_hori_p + err_pitch * euler_hori_i

            err_yaw = _bound(exp_yaw[0], euler_vert_max_angle) - theta[2]
            out_yaw = - omega[2] * euler_vert_p + err_yaw * euler_vert_i

            # ## vertical control ## #
            err_h = exp_pos[2] - pos[2]
            err_vh = -vel[2]
            out_h = err_vh * pos_vert_p + err_h * pos_vert_i + self.__aero_mass * self.__gravity / 4 / 0.4
            out_h = out_h / cosd(theta[0]) / cosd(theta[1])

            lf = out_h + out_roll - out_pitch + out_yaw
            lb = out_h + out_roll + out_pitch - out_yaw
            rb = out_h - out_roll + out_pitch + out_yaw
            rf = out_h - out_roll - out_pitch - out_yaw
            # print(GREEN + "err_h: {}, err_vh: {}, out_h: {}".format(err_h, err_vh, out_h) + RESET)

        self.model_evaluate(lf, lb, rb, rf)

