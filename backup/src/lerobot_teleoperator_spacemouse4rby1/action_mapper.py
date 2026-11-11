"""Convert dual-arm pose targets to RBY1 native EE actions."""

import numpy as np
from scipy.spatial.transform import Rotation

SIDES = ("right", "left")
EE_SUFFIXES = ("x", "y", "z", "wx", "wy", "wz")


class Rby1ActionMapper:
    def __init__(self, use_gripper=True):
        self.use_gripper = use_gripper

    @property
    def action_features(self):
        features = {
            f"{side}_ee.{suffix}": float
            for side in SIDES
            for suffix in EE_SUFFIXES
        }

        if self.use_gripper:
            features.update({
                f"{side}_gripper_0.pos": float
                for side in SIDES
            })

        return features

    def __call__(self, poses, grippers=None):
        action = {}

        for side in SIDES:
            pose = np.asarray(poses[side], dtype=float)

            if (
                pose.shape != (4, 4)
                or not np.isfinite(pose).all()
                or not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-6)
            ):
                raise ValueError(f"Invalid {side} pose.")

            rotation = pose[:3, :3]

            if (
                not np.allclose(
                    rotation.T @ rotation,
                    np.eye(3),
                    atol=1e-6,
                )
                or not np.isclose(
                    np.linalg.det(rotation),
                    1.0,
                    atol=1e-6,
                )
            ):
                raise ValueError(f"Invalid {side} rotation matrix.")

            rotvec = Rotation.from_matrix(rotation).as_rotvec()
            values = (*pose[:3, 3], *rotvec)

            action.update({
                f"{side}_ee.{suffix}": float(value)
                for suffix, value in zip(EE_SUFFIXES, values)
            })

        if self.use_gripper:
            if grippers is None:
                raise ValueError("Gripper targets are required.")

            for side in SIDES:
                value = float(grippers[side])

                if not np.isfinite(value) or not 0.0 <= value <= 1.0:
                    raise ValueError(f"Invalid {side} gripper target.")

                action[f"{side}_gripper_0.pos"] = value

        return action