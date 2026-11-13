import numpy as np


def get_gravity_orientation(quaternion):
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3)

    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

    return gravity_orientation


def get_yaw(quaternion):
    qw, qx, qy, qz = quaternion
    return np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def quat_from_yaw(angle):
    """Quaternion (w, x, y, z) of a rotation about the vertical axis."""
    return np.array([np.cos(angle / 2), 0.0, 0.0, np.sin(angle / 2)])


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd
