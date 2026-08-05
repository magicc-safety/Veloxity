#!/usr/bin/env python3
"""Dependency-free math used by the mocap state publisher."""

import math
from dataclasses import dataclass


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def subtract(a, b):
    return tuple(x - y for x, y in zip(a, b))


def scale(a, scalar):
    return tuple(scalar * x for x in a)


def norm(a):
    return math.sqrt(sum(x * x for x in a))


def quaternion_normalize(q):
    magnitude = norm(q)
    if not math.isfinite(magnitude) or magnitude < 1.0e-9:
        raise ValueError("quaternion has zero or non-finite norm")
    return tuple(x / magnitude for x in q)


def quaternion_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quaternion_multiply(left, right):
    """Compose xyzw quaternions, applying right and then left."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_rotate(q, vector):
    q = quaternion_normalize(q)
    vector_q = (vector[0], vector[1], vector[2], 0.0)
    rotated = quaternion_multiply(
        quaternion_multiply(q, vector_q), quaternion_conjugate(q)
    )
    return rotated[:3]


def quaternion_angle(left, right):
    left = quaternion_normalize(left)
    right = quaternion_normalize(right)
    dot = abs(sum(x * y for x, y in zip(left, right)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def quaternion_to_euler(q):
    """Return aerospace roll, pitch, yaw for a body-to-NED quaternion."""
    x, y, z, w = quaternion_normalize(q)
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def ned_pose_to_room_marker(
    position_ned, body_q_ned, room_to_ned_q, body_to_marker_q
):
    """Invert the room-marker to NED-body transform."""
    position_room = quaternion_rotate(
        quaternion_conjugate(room_to_ned_q), position_ned
    )
    room_marker_q = quaternion_normalize(
        quaternion_multiply(
            quaternion_multiply(
                quaternion_conjugate(room_to_ned_q), body_q_ned
            ),
            quaternion_conjugate(body_to_marker_q),
        )
    )
    return position_room, room_marker_q


@dataclass
class AlphaBetaFilter3D:
    """Acceleration-aided alpha-beta tracker for a three-dimensional position.

    Passing no acceleration preserves the original constant-velocity behavior.
    """

    alpha: float
    beta: float
    position: tuple = (0.0, 0.0, 0.0)
    velocity: tuple = (0.0, 0.0, 0.0)
    initialized: bool = False

    def __post_init__(self):
        if not 0.0 < self.alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        if not 0.0 < self.beta <= 2.0:
            raise ValueError("beta must be in (0, 2]")

    def reset(self, measurement):
        self.position = tuple(float(x) for x in measurement)
        self.velocity = (0.0, 0.0, 0.0)
        self.initialized = True

    def predicted_state(self, dt, acceleration=(0.0, 0.0, 0.0)):
        if not self.initialized:
            raise RuntimeError("filter is not initialized")
        if not math.isfinite(dt) or dt < 0.0:
            raise ValueError("filter dt must be non-negative and finite")
        predicted_position = add(
            add(self.position, scale(self.velocity, dt)),
            scale(acceleration, 0.5 * dt * dt),
        )
        predicted_velocity = add(self.velocity, scale(acceleration, dt))
        return predicted_position, predicted_velocity

    def predicted_position(self, dt, acceleration=(0.0, 0.0, 0.0)):
        return self.predicted_state(dt, acceleration)[0]

    def innovation(self, measurement, dt, acceleration=(0.0, 0.0, 0.0)):
        return subtract(measurement, self.predicted_position(dt, acceleration))

    def update(self, measurement, dt, acceleration=(0.0, 0.0, 0.0)):
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("filter dt must be positive and finite")
        if not self.initialized:
            self.reset(measurement)
            return self.position, self.velocity

        predicted_position, predicted_velocity = self.predicted_state(
            dt, acceleration
        )
        residual = subtract(measurement, predicted_position)
        self.position = add(predicted_position, scale(residual, self.alpha))
        self.velocity = add(predicted_velocity, scale(residual, self.beta / dt))
        return self.position, self.velocity
