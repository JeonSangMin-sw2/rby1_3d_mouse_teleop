"""Read RB-Y1 arm poses through the SDK without sending commands."""

from contextlib import suppress

import numpy as np
import rby1_sdk as rby


class Rby1StateReader:
    def __init__(
        self,
        address: str = "192.168.30.1:50051",
        model: str = "auto",
    ):
        self.address = address
        self.model = model.strip().lower()

        if self.model not in ("auto", "a", "m", "ub"):
            raise ValueError("model must be auto, a, m, or ub.")

        self.resolved_model = None
        self._robot = None
        self._dynamics = None
        self._fk_state = None
        self._joint_count = 0

    def _detect_model(self) -> str:
        # 기존 RBY1 플러그인과 같은 방식으로 모델 정보만 조회한다.
        probe = rby.create_robot_a(self.address)

        try:
            if not probe.connect(max_retries=1, timeout_ms=3000):
                raise ConnectionError(
                    f"Cannot connect to {self.address}."
                )

            name = (
                probe.get_robot_info()
                .robot_model_name.strip()
                .lower()
            )

            for model in ("a", "m", "ub"):
                if name.startswith(model):
                    return model

            raise ValueError(f"Unknown robot model: {name!r}")

        finally:
            with suppress(Exception):
                probe.disconnect()

    def connect(self):
        if self._robot is not None:
            raise RuntimeError("State reader is already connected.")

        model = (
            self._detect_model()
            if self.model == "auto"
            else self.model
        )
        self._robot = rby.create_robot(self.address, model)

        try:
            if not self._robot.connect(
                max_retries=1,
                timeout_ms=3000,
            ):
                raise ConnectionError(
                    f"Cannot connect to {self.address}."
                )

            robot_model = self._robot.model()
            self._joint_count = len(robot_model.robot_joint_names)
            self._dynamics = self._robot.get_dynamics()

            # 아래 link 목록의 순서가 transformation index가 된다.
            # 0: base, 1: right arm, 2: left arm
            self._fk_state = self._dynamics.make_state(
                ["base", "link_right_arm_6", "link_left_arm_6"],
                robot_model.robot_joint_names,
            )

            self.resolved_model = model

        except BaseException:
            with suppress(Exception):
                self.disconnect()
            raise

    def read(self) -> dict[str, np.ndarray]:
        if self._robot is None or self._fk_state is None:
            raise RuntimeError("Call connect() before read().")

        state = self._robot.get_state()
        q = np.asarray(
            state.position,
            dtype=np.float64,
        ).copy()

        if (
            q.shape != (self._joint_count,)
            or not np.isfinite(q).all()
        ):
            raise ValueError("Invalid robot joint-position state.")

        self._fk_state.set_q(q)
        self._dynamics.compute_forward_kinematics(self._fk_state)

        poses = {}

        for side, link_index in (("right", 1), ("left", 2)):
            pose = np.array(
                self._dynamics.compute_transformation(
                    self._fk_state,
                    0,
                    link_index,
                ),
                dtype=np.float64,
                copy=True,
            )

            if (
                pose.shape != (4, 4)
                or not np.isfinite(pose).all()
            ):
                raise ValueError(f"Invalid {side} arm pose.")

            poses[side] = pose

        return poses

    def disconnect(self):
        robot = self._robot

        self._robot = None
        self._dynamics = None
        self._fk_state = None
        self._joint_count = 0
        self.resolved_model = None

        if robot is not None:
            robot.disconnect()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--address",
        default="127.0.0.1:50051",
    )
    parser.add_argument(
        "--model",
        choices=("auto", "a", "m", "ub"),
        default="auto",
    )
    args = parser.parse_args()

    np.set_printoptions(precision=4, suppress=True)

    with Rby1StateReader(args.address, args.model) as reader:
        print(f"Robot model: {reader.resolved_model}")

        poses = reader.read()

        for side in ("left", "right"):
            print(f"{side} pose (base -> arm_6):")
            print(poses[side])
            print(
                f"{side} position [m]: "
                f"{poses[side][:3, 3]}"
            )