#!/usr/bin/env python3

import numpy as np
import math


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class EKFPlanarIMU:


    DIM_X = 9
    I_PX, I_PY, I_Z, I_YAW = 0, 1, 2, 3
    I_VX, I_VY = 4, 5
    I_BAX, I_BAY, I_BGZ = 6, 7, 8

    def __init__(self, dt, process_noise_diag=None, initial_cov_diag=None):
        self.dt = dt
        self.dim_x = self.DIM_X
        self.nis_gate_default = 16.0

        self.x = np.zeros(self.dim_x)

        if initial_cov_diag is None:
            self.P = np.eye(self.dim_x) * 0.5
        else:
            d = np.asarray(initial_cov_diag, dtype=float).reshape(-1)
            if d.size != self.dim_x:
                raise ValueError(
                    f"initial_cov_diag must have length {self.dim_x}, got {d.size}"
                )
            self.P = np.diag(d)

        if process_noise_diag is None:
            self.q_diag = np.array([
                1e-4, 1e-4, 1e-6, 1e-6,
                5e-3, 5e-3,
                1e-8, 1e-8, 1e-10,
            ], dtype=float)
        else:
            self.q_diag = np.asarray(process_noise_diag, dtype=float).reshape(-1)
            if self.q_diag.size != self.dim_x:
                raise ValueError(
                    f"process_noise_diag must have length {self.dim_x}, got {self.q_diag.size}"
                )

        self.use_linear_accel = True

    def set_process_noise(self, process_noise_diag):
        d = np.asarray(process_noise_diag, dtype=float).reshape(-1)
        if d.size != self.dim_x:
            raise ValueError(
                f"process_noise_diag must have length {self.dim_x}, got {d.size}"
            )
        self.q_diag = d

    def predict(self, acc_meas, gyro_meas, dt=None):
        if dt is None:
            dt = self.dt

        x = self.x.copy()
        px, py, z = x[self.I_PX], x[self.I_PY], x[self.I_Z]
        yaw = x[self.I_YAW]
        vx, vy = x[self.I_VX], x[self.I_VY]
        bax, bay, bgz = x[self.I_BAX], x[self.I_BAY], x[self.I_BGZ]

        w = float(gyro_meas[2]) - bgz
        yaw_n = wrap_angle(yaw + w * dt)

        c = math.cos(yaw)
        s = math.sin(yaw)

        if self.use_linear_accel:
            ax_b = float(acc_meas[0]) - bax
            ay_b = float(acc_meas[1]) - bay
            ax_w = c * ax_b - s * ay_b
            ay_w = s * ax_b + c * ay_b

            vx_n = vx + ax_w * dt
            vy_n = vy + ay_w * dt
            px_n = px + vx * dt + 0.5 * ax_w * dt * dt
            py_n = py + vy * dt + 0.5 * ay_w * dt * dt

            self.x[self.I_PX] = px_n
            self.x[self.I_PY] = py_n
            self.x[self.I_Z] = z
            self.x[self.I_YAW] = yaw_n
            self.x[self.I_VX] = vx_n
            self.x[self.I_VY] = vy_n

            F = np.eye(self.dim_x)

            daxw_dyaw = -s * ax_b - c * ay_b
            dayw_dyaw = c * ax_b - s * ay_b

            F[self.I_PX, self.I_VX] = dt
            F[self.I_PY, self.I_VY] = dt

            F[self.I_PX, self.I_YAW] = 0.5 * dt * dt * daxw_dyaw
            F[self.I_PY, self.I_YAW] = 0.5 * dt * dt * dayw_dyaw

            F[self.I_PX, self.I_BAX] = 0.5 * dt * dt * (-c)
            F[self.I_PX, self.I_BAY] = 0.5 * dt * dt * (s)
            F[self.I_PY, self.I_BAX] = 0.5 * dt * dt * (-s)
            F[self.I_PY, self.I_BAY] = 0.5 * dt * dt * (-c)

            F[self.I_YAW, self.I_BGZ] = -dt

            F[self.I_VX, self.I_YAW] = dt * daxw_dyaw
            F[self.I_VY, self.I_YAW] = dt * dayw_dyaw

            F[self.I_VX, self.I_BAX] = dt * (-c)
            F[self.I_VX, self.I_BAY] = dt * (s)
            F[self.I_VY, self.I_BAX] = dt * (-s)
            F[self.I_VY, self.I_BAY] = dt * (-c)
        else:
            vx_n = vx
            vy_n = vy
            px_n = px + vx * dt
            py_n = py + vy * dt

            self.x[self.I_PX] = px_n
            self.x[self.I_PY] = py_n
            self.x[self.I_Z] = z
            self.x[self.I_YAW] = yaw_n
            self.x[self.I_VX] = vx_n
            self.x[self.I_VY] = vy_n

            F = np.eye(self.dim_x)
            F[self.I_PX, self.I_VX] = dt
            F[self.I_PY, self.I_VY] = dt
            F[self.I_YAW, self.I_BGZ] = -dt

        Q = np.diag(self.q_diag) * dt
        self.P = F @ self.P @ F.T + Q

    def update_lidar_pose(
        self,
        px,
        py,
        z,
        roll,
        pitch,
        yaw,
        var=0.05,
        var_yaw=None,
        use_roll_pitch=False,
        gate_nis=None,
    ):
        if use_roll_pitch:
            raise NotImplementedError(
                "Planar EKF does not fuse roll/pitch; set lidar_use_roll_pitch: false"
            )
        z_meas = np.array([px, py, z, yaw])
        H = np.zeros((4, self.dim_x))
        H[0, self.I_PX] = 1
        H[1, self.I_PY] = 1
        H[2, self.I_Z] = 1
        H[3, self.I_YAW] = 1
        v = float(var)
        vy = float(var_yaw) if var_yaw is not None else v
        R = np.diag([v, v, v, vy])
        return self._update(z_meas, H, R, angle_idx=[3], gate_nis=gate_nis)

    def nis_lidar_xy(self, px, py, var=0.05):

        z_meas = np.array([float(px), float(py)], dtype=float)
        H = np.zeros((2, self.dim_x))
        H[0, self.I_PX] = 1.0
        H[1, self.I_PY] = 1.0
        z_pred = H @ self.x
        y = z_meas - z_pred
        v = float(var)
        R = np.diag([v, v])
        S = H @ self.P @ H.T + R
        return float(y.T @ np.linalg.inv(S) @ y)

    def nis_lidar_xy_yaw(self, px, py, yaw, var=0.05, var_yaw=None):

        z_meas = np.array([float(px), float(py), float(yaw)], dtype=float)
        H = np.zeros((3, self.dim_x))
        H[0, self.I_PX] = 1.0
        H[1, self.I_PY] = 1.0
        H[2, self.I_YAW] = 1.0
        z_pred = H @ self.x
        y = z_meas - z_pred
        y[2] = wrap_angle(y[2])
        v = float(var)
        vy = float(var_yaw) if var_yaw is not None else v
        R = np.diag([v, v, vy])
        S = H @ self.P @ H.T + R
        return float(y.T @ np.linalg.inv(S) @ y)

    def update_lidar_xy(self, px, py, var=0.05, gate_nis=None):

        z_meas = np.array([float(px), float(py)], dtype=float)
        H = np.zeros((2, self.dim_x))
        H[0, self.I_PX] = 1.0
        H[1, self.I_PY] = 1.0
        v = float(var)
        R = np.diag([v, v])
        return self._update(z_meas, H, R, angle_idx=None, gate_nis=gate_nis)

    def update_lidar_xy_yaw(
        self, px, py, yaw, var=0.05, var_yaw=None, gate_nis=None
    ):

        z_meas = np.array([px, py, yaw])
        H = np.zeros((3, self.dim_x))
        H[0, self.I_PX] = 1
        H[1, self.I_PY] = 1
        H[2, self.I_YAW] = 1
        v = float(var)
        vy = float(var_yaw) if var_yaw is not None else v
        R = np.diag([v, v, vy])
        return self._update(z_meas, H, R, angle_idx=[2], gate_nis=gate_nis)

    def update_lidar_velocity_xy(self, vx, vy, var=0.25, gate_nis=None):

        z_meas = np.array([float(vx), float(vy)], dtype=float)
        H = np.zeros((2, self.dim_x))
        H[0, self.I_VX] = 1.0
        H[1, self.I_VY] = 1.0
        v = float(var)
        R = np.diag([v, v])
        return self._update(z_meas, H, R, angle_idx=None, gate_nis=gate_nis)

    def update_position(self, px, py, z, var=0.05, gate_nis=None):
        z_meas = np.array([px, py, z])
        H = np.zeros((3, self.dim_x))
        H[0, self.I_PX] = 1
        H[1, self.I_PY] = 1
        H[2, self.I_Z] = 1
        R = self._make_R(var, 3)
        return self._update(z_meas, H, R, gate_nis=gate_nis)

    def update_lidar_z(self, z, var=0.05, gate_nis=None):
        z_meas = np.array([z])
        H = np.zeros((1, self.dim_x))
        H[0, self.I_Z] = 1
        R = self._make_R(var, 1)
        return self._update(z_meas, H, R, gate_nis=gate_nis)

    def _update(self, z, H, R, angle_idx=None, gate_nis=None):
        z_pred = H @ self.x
        y = z - z_pred

        if angle_idx is not None:
            for i in angle_idx:
                y[i] = wrap_angle(y[i])

        S = H @ self.P @ H.T + R
        if gate_nis is not None:
            nis = float(y.T @ np.linalg.inv(S) @ y)
            if nis > gate_nis:
                return False

        K = self.P @ H.T @ np.linalg.inv(S)
        ident = np.eye(self.dim_x)
        self.x = self.x + K @ y
        self.x[self.I_YAW] = wrap_angle(self.x[self.I_YAW])

        self.P = (ident - K @ H) @ self.P @ (ident - K @ H).T + K @ R @ K.T
        return True

    def _make_R(self, var, n):
        if np.isscalar(var):
            return np.eye(n) * float(var)
        arr = np.asarray(var, dtype=float).reshape(-1)
        if arr.size != n:
            raise ValueError(f"Expected var length {n}, got {arr.size}")
        return np.diag(arr)

    def get_state(self):
        return self.x.copy()

    def get_pose(self):

        pos = self.x[0:3].copy()
        rpy = np.array([0.0, 0.0, float(self.x[self.I_YAW])])
        return pos, rpy

    def get_biases(self):

        return (
            float(self.x[self.I_BAX]),
            float(self.x[self.I_BAY]),
            float(self.x[self.I_BGZ]),
        )


EKF_IMU_LIDAR = EKFPlanarIMU
