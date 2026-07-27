"""ROS-independent observation and ONNX inference logic for SEA-Nav."""

import math
from collections import deque

import numpy as np
import onnxruntime as ort


HISTORY_LENGTH = 10
NUM_PROPERTIES = 12
NUM_RAYS = 41
GOAL_SIZE = 2
OBSERVATION_STEP_SIZE = NUM_PROPERTIES + NUM_RAYS + GOAL_SIZE
OBSERVATION_SIZE = HISTORY_LENGTH * OBSERVATION_STEP_SIZE


def normalize_quaternion(quaternion):
    quaternion = np.asarray(quaternion, dtype=np.float32)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-8:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    return quaternion / norm


def quaternion_conjugate(quaternion):
    x, y, z, w = quaternion
    return np.array([-x, -y, -z, w], dtype=np.float32)


def rotate_vector(quaternion, vector):
    """Rotate a 3-D vector by an xyzw quaternion."""
    quaternion = normalize_quaternion(quaternion)
    vector = np.asarray(vector, dtype=np.float32)
    uv = np.cross(quaternion[:3], vector)
    uuv = np.cross(quaternion[:3], uv)
    return vector + 2.0 * (quaternion[3] * uv + uuv)


def yaw_from_quaternion(quaternion):
    x, y, z, w = normalize_quaternion(quaternion)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class SeaNavGo2Policy:
    """Build training-compatible history and run the navigation policy."""

    def __init__(
        self,
        model_path,
        ray_angle_min=-2.0 * math.pi / 3.0,
        ray_angle_max=2.0 * math.pi / 3.0,
        ray_min=0.1,
        ray_max=5.0,
        action_alpha=0.5,
        action_min=(-0.5, -1.0, -1.0),
        action_max=(2.0, 1.0, 1.0),
    ):
        self.ray_angles = np.linspace(
            ray_angle_min, ray_angle_max, NUM_RAYS, dtype=np.float32
        )
        self.ray_min = float(ray_min)
        self.ray_max = float(ray_max)
        self.action_alpha = float(action_alpha)
        self.action_min = np.asarray(action_min, dtype=np.float32)
        self.action_max = np.asarray(action_max, dtype=np.float32)
        self.history = deque(maxlen=HISTORY_LENGTH)
        self.filtered_action = np.zeros(3, dtype=np.float32)

        self.session = ort.InferenceSession(
            str(model_path), providers=["CPUExecutionProvider"]
        )
        model_input = self.session.get_inputs()[0]
        model_output = self.session.get_outputs()[0]
        if model_input.shape[-1] != OBSERVATION_SIZE or model_output.shape[-1] != 3:
            raise RuntimeError(
                "ONNX shape mismatch: expected [batch, 550] -> [batch, 3], "
                f"got {model_input.shape} -> {model_output.shape}"
            )
        self.input_name = model_input.name

    def reset_history(self, reset_action=False):
        self.history.clear()
        if reset_action:
            self.filtered_action.fill(0.0)

    def sample_scan(self, ranges, angle_min, angle_increment):
        """Resample an arbitrary LaserScan to the 41 training ray angles."""
        source_ranges = np.asarray(ranges, dtype=np.float32)
        if source_ranges.size < 2 or angle_increment == 0.0:
            raise ValueError("LaserScan must contain at least two angular samples")
        source_angles = angle_min + np.arange(
            source_ranges.size, dtype=np.float32
        ) * angle_increment
        if source_angles[0] > source_angles[-1]:
            source_angles = source_angles[::-1]
            source_ranges = source_ranges[::-1]
        source_ranges = np.nan_to_num(
            source_ranges,
            nan=self.ray_min,
            posinf=self.ray_max,
            neginf=self.ray_min,
        )
        source_ranges = np.clip(source_ranges, self.ray_min, self.ray_max)

        # Outside the sensor FOV is unknown, so expose it as occupied rather
        # than telling the policy that unobserved space is clear.
        return np.interp(
            self.ray_angles,
            source_angles,
            source_ranges,
            left=self.ray_min,
            right=self.ray_min,
        ).astype(np.float32)

    def build_observation(
        self,
        position,
        orientation,
        linear_velocity,
        angular_velocity,
        rays,
        goal,
        twist_in_body_frame,
    ):
        orientation = normalize_quaternion(orientation)
        inverse_orientation = quaternion_conjugate(orientation)
        projected_gravity = rotate_vector(
            inverse_orientation, [0.0, 0.0, -1.0]
        )

        if twist_in_body_frame:
            body_linear = np.asarray(linear_velocity, dtype=np.float32)
            body_angular = np.asarray(angular_velocity, dtype=np.float32)
        else:
            body_linear = rotate_vector(inverse_orientation, linear_velocity)
            body_angular = rotate_vector(inverse_orientation, angular_velocity)

        delta_world = np.asarray(goal, dtype=np.float32) - np.asarray(
            position[:2], dtype=np.float32
        )
        yaw = yaw_from_quaternion(orientation)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        goal_local = np.array(
            [
                cosine * delta_world[0] + sine * delta_world[1],
                -sine * delta_world[0] + cosine * delta_world[1],
            ],
            dtype=np.float32,
        )

        # Keep this order identical to training: gravity(3), previous filtered
        # command(3), body linear velocity(3), body angular velocity(3),
        # log2 lidar rays(41), and local goal xy(2).
        properties = np.concatenate(
            (
                projected_gravity,
                self.filtered_action,
                body_linear,
                body_angular,
            )
        ).astype(np.float32)
        ray_observation = np.log2(
            np.clip(rays, self.ray_min, self.ray_max)
        ).astype(np.float32)
        observation = np.concatenate(
            (properties, ray_observation, goal_local)
        ).astype(np.float32)
        if observation.shape != (OBSERVATION_STEP_SIZE,):
            raise RuntimeError(
                f"One-step observation has invalid shape {observation.shape}"
            )
        return observation

    def infer(self, observation):
        if not self.history:
            # Isaac Gym fills every history slot with the first post-reset
            # observation. Zero padding here would create an unseen input.
            for _ in range(HISTORY_LENGTH):
                self.history.append(observation.copy())
        else:
            self.history.append(observation.copy())

        model_input = np.concatenate(tuple(self.history), axis=0)[None, :]
        raw_action = self.session.run(
            None, {self.input_name: model_input}
        )[0][0].astype(np.float32)
        if not np.isfinite(raw_action).all():
            raise RuntimeError("ONNX policy produced NaN or Inf")

        filtered = (
            self.action_alpha * raw_action
            + (1.0 - self.action_alpha) * self.filtered_action
        )
        self.filtered_action = np.clip(
            filtered, self.action_min, self.action_max
        ).astype(np.float32)
        return raw_action, self.filtered_action.copy()
