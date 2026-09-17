"""M20 control: joint configuration, observation/action mapping and ONNX inference.

Copy the whole policy directory to use this controller without Isaac Sim.
"""
from __future__ import annotations
from pathlib import Path
from typing import Mapping, Sequence
import hashlib
import json
import numpy as np
import onnxruntime as ort

LEGS = ("fl", "fr", "hl", "hr")
ROBOT_ORDER = tuple(f"{leg}_{joint}_joint" for leg in LEGS for joint in ("hipx", "hipy", "knee", "wheel"))

POLICY_ORDER = tuple(f"{leg}_{joint}_joint" for leg in LEGS for joint in ("hipx", "hipy", "knee")) + tuple(f"{leg}_wheel_joint" for leg in LEGS)

PIPER_JOINTS: tuple[str, ...] = tuple(f"joint{index}" for index in range(1, 9))

EXPECTED_DOF_NAMES = frozenset((*ROBOT_ORDER, *PIPER_JOINTS))

LEG_JOINTS: tuple[str, ...] = POLICY_ORDER[:12]

WHEEL_JOINTS: tuple[str, ...] = POLICY_ORDER[12:]

DEFAULT_POSITION_BY_NAME = {
    f"{leg}_{joint}_joint": value
    for leg in LEGS
    for joint, value in zip(("hipx", "hipy", "knee", "wheel"),
                            (0.0, -0.3, 0.6, 0.0) if leg in ("fl", "fr") else (0.0, 0.3, -0.6, 0.0))
}

ACTION_SCALE_BY_NAME: Mapping[str, float] = {
    **{name: (0.125 if "hipx" in name else 0.25) for name in LEG_JOINTS},
    **{name: 5.0 for name in WHEEL_JOINTS},
}

KP_BY_NAME: Mapping[str, float] = {
    **{name: 80.0 for name in LEG_JOINTS},
    **{name: 0.0 for name in WHEEL_JOINTS},
    **{name: 400.0 for name in PIPER_JOINTS[:3]},
    **{name: 200.0 for name in PIPER_JOINTS[3:6]},
    **{name: 800.0 for name in PIPER_JOINTS[6:]},
}

KD_BY_NAME: Mapping[str, float] = {
    **{name: 2.0 for name in LEG_JOINTS},
    **{name: 0.6 for name in WHEEL_JOINTS},
    **{name: 80.0 for name in PIPER_JOINTS[:3]},
    **{name: 40.0 for name in PIPER_JOINTS[3:6]},
    **{name: 20.0 for name in PIPER_JOINTS[6:]},
}

EFFORT_LIMIT_BY_NAME: Mapping[str, float] = {
    **{name: 76.4 for name in LEG_JOINTS},
    **{name: 21.6 for name in WHEEL_JOINTS},
    **{name: 100.0 for name in PIPER_JOINTS[:6]},
    **{name: 10.0 for name in PIPER_JOINTS[6:]},
}

ARMATURE_BY_NAME: Mapping[str, float] = {
    **{name: 0.0 for name in LEG_JOINTS},
    **{name: 0.00243216 for name in WHEEL_JOINTS},
}

def quaternion_to_rotation_matrix(quaternion_wxyz: Sequence[float]) -> np.ndarray:
    quaternion = np.asarray(quaternion_wxyz, dtype=np.float64)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("quaternion must be a finite wxyz vector")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-12:
        raise ValueError("quaternion norm is zero")
    w, x, y, z = quaternion / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )

def projected_gravity(quaternion_wxyz: Sequence[float]) -> np.ndarray:
    """Return unit gravity in the robot base frame."""

    rotation_world_from_body = quaternion_to_rotation_matrix(quaternion_wxyz)
    return (rotation_world_from_body.T @ np.asarray([0.0, 0.0, -1.0], dtype=np.float64)).astype(np.float32)

def _vector(value: Sequence[float] | np.ndarray, length: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must be a finite vector with shape ({length},)")
    return result

def build_policy_observation(
    base_angular_velocity_xyz: Sequence[float],
    gravity_body_xyz: Sequence[float],
    command_vx_vy_wz: Sequence[float],
    joint_positions_robot_order: Sequence[float],
    joint_velocities_robot_order: Sequence[float],
    previous_action_policy_order: Sequence[float],
) -> np.ndarray:
    """Build the official M20 ONNX observation with shape ``[1, 57]``."""

    angular_velocity = _vector(base_angular_velocity_xyz, 3, "base_angular_velocity_xyz") * 0.25
    gravity = _vector(gravity_body_xyz, 3, "gravity_body_xyz")
    command = _vector(command_vx_vy_wz, 3, "command_vx_vy_wz")
    position_robot = _vector(joint_positions_robot_order, 16, "joint_positions_robot_order")
    velocity_robot = _vector(joint_velocities_robot_order, 16, "joint_velocities_robot_order")
    previous_action = _vector(previous_action_policy_order, 16, "previous_action_policy_order")

    robot_index = {name: index for index, name in enumerate(ROBOT_ORDER)}
    position_policy = np.asarray([position_robot[robot_index[name]] for name in POLICY_ORDER], dtype=np.float32)
    velocity_policy = np.asarray([velocity_robot[robot_index[name]] for name in POLICY_ORDER], dtype=np.float32)
    default_policy = np.asarray([DEFAULT_POSITION_BY_NAME[name] for name in POLICY_ORDER], dtype=np.float32)
    position_policy[12:] = 0.0
    relative_position = position_policy - default_policy
    observation = np.concatenate(
        (
            angular_velocity,
            gravity,
            command,
            relative_position,
            velocity_policy * 0.05,
            previous_action,
        )
    ).astype(np.float32, copy=False)
    if observation.shape != (57,) or not np.isfinite(observation).all():
        raise RuntimeError("constructed policy observation is not finite [57]")
    return observation.reshape(1, 57)

def policy_action_to_targets(action_policy_order: Sequence[float]) -> tuple[dict[str, float], dict[str, float]]:
    """Map official policy output to 12 position and four velocity targets."""

    action = _vector(action_policy_order, 16, "action_policy_order")
    position_targets = {
        name: DEFAULT_POSITION_BY_NAME[name] + float(action[index]) * ACTION_SCALE_BY_NAME[name]
        for index, name in enumerate(LEG_JOINTS)
    }
    velocity_targets = {
        name: float(action[index + 12]) * ACTION_SCALE_BY_NAME[name]
        for index, name in enumerate(WHEEL_JOINTS)
    }
    return position_targets, velocity_targets


class M20Policy:
    """Input joint arrays use ROBOT_ORDER; quaternion uses wxyz, angular velocity world frame."""
    def __init__(self, policy_dir=None):
        self.directory = Path(policy_dir) if policy_dir else Path(__file__).parent
        self.config = json.loads((self.directory / 'config.json').read_text())
        model = self.directory / 'policy.onnx'
        if hashlib.sha256(model.read_bytes()).hexdigest() != self.config['sha256']:
            raise ValueError('policy.onnx hash differs from config.json')
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(model), options, providers=['CPUExecutionProvider'])
        ins, outs = self.session.get_inputs(), self.session.get_outputs()
        if len(ins) != 1 or ins[0].name != 'obs' or ins[0].shape != [1, 57] or ins[0].type != 'tensor(float)':
            raise ValueError('Expected float32 obs [1,57]')
        if len(outs) != 1 or outs[0].name != 'actions' or outs[0].shape != [1, 16] or outs[0].type != 'tensor(float)':
            raise ValueError('Expected float32 actions [1,16]')
        self.reset()

    def reset(self):
        self.previous_action = np.zeros(16, dtype=np.float32)
        self.calls = 0

    def step(self, quaternion_wxyz, angular_velocity_world, joint_positions, joint_velocities, command):
        command = np.asarray(command, dtype=np.float32)
        limits = np.asarray(self.config['command_limits_vx_vy_wz'])
        if command.shape != (3,) or not np.isfinite(command).all() or np.any(np.abs(command) > limits + 1e-6):
            raise ValueError('Command must be finite [vx,vy,wz] within config limits')
        rotation = quaternion_to_rotation_matrix(quaternion_wxyz)
        obs = build_policy_observation(
            rotation.T @ np.asarray(angular_velocity_world), projected_gravity(quaternion_wxyz),
            command, joint_positions, joint_velocities, self.previous_action,
        )
        action = np.asarray(self.session.run(['actions'], {'obs': obs})[0], dtype=np.float32)
        if action.shape != (1, 16) or not np.isfinite(action).all():
            raise RuntimeError('Non-finite or malformed ONNX action')
        positions, velocities = policy_action_to_targets(action[0])
        self.previous_action = action[0].copy()
        self.calls += 1
        return positions, velocities

