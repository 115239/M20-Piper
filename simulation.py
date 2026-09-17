#!/usr/bin/env python3
"""Self-contained M20 + Piper flat-ground example for Isaac Sim 5.1."""
from pathlib import Path
import argparse
import hashlib
import json
import math
import sys

ROOT = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--headless', action='store_true')
parser.add_argument('--seconds', type=float, default=20)
parser.add_argument('--vx', type=float, default=0.1)
parser.add_argument('--vy', type=float, default=0)
parser.add_argument('--wz', type=float, default=0)
parser.add_argument('--policy-dir', type=Path, default=ROOT / 'policy')
parser.add_argument('--output', type=Path, default=ROOT / 'outputs/simulation')
parser.add_argument('--arm-demo', action='store_true', help='Small joint1 and gripper motion with zero base command only')
args = parser.parse_args()
if not math.isfinite(args.seconds) or args.seconds < 6:
    parser.error('--seconds must be finite and at least 6 (includes settling and stop)')
if args.arm_demo and any((args.vx, args.vy, args.wz)):
    parser.error('--arm-demo requires --vx 0 --vy 0 --wz 0')
sys.path.insert(0, str(ROOT / 'policy'))
import numpy as np
from controller import (
    M20Policy,
    ROBOT_ORDER, WHEEL_JOINTS, PIPER_JOINTS, EXPECTED_DOF_NAMES,
    DEFAULT_POSITION_BY_NAME, KP_BY_NAME, KD_BY_NAME, EFFORT_LIMIT_BY_NAME,
    ARMATURE_BY_NAME, quaternion_to_rotation_matrix,
)
policy = M20Policy(args.policy_dir)
command = np.array([args.vx, args.vy, args.wz], dtype=np.float32)
if not np.isfinite(command).all() or np.any(np.abs(command) > np.array(policy.config['command_limits_vx_vy_wz']) + 1e-6):
    parser.error('Requested command exceeds policy config limits')
from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless, 'width': 1280, 'height': 720})
report = {'status': 'RUNNING', 'scope': 'flat-ground simulation only', 'policy_sha256': policy.config['sha256'],
          'runner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          'requested_seconds': args.seconds, 'command_vx_vy_wz': command.tolist(), 'arm_demo': args.arm_demo}
args.output.mkdir(parents=True, exist_ok=True)
try:
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import GroundPlane
    from isaacsim.core.api.materials import PhysicsMaterial
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import UsdLux, Gf
    import omni.usd

    world = World(physics_dt=0.005, rendering_dt=0.005, stage_units_in_meters=1.0, backend='numpy', device='cpu')
    ground_material = PhysicsMaterial('/World/GroundMaterial', static_friction=0.8,
                                      dynamic_friction=0.8, restitution=0.0)
    world.scene.add(GroundPlane('/World/Ground', size=100, z_position=0,
                               color=np.array([0.35, 0.38, 0.42]), physics_material=ground_material))
    stage = omni.usd.get_context().get_stage()
    light = UsdLux.DomeLight.Define(stage, '/World/EnvironmentLight')
    light.CreateIntensityAttr(700)
    robot_path = '/World/Robot'
    asset = ROOT / 'model/piper_piper_d435i_sensors.usda'
    add_reference_to_stage(str(asset), robot_path)
    robot = world.scene.add(SingleArticulation(prim_path=robot_path, name='m20_piper', position=np.array([0., 0., 0.58])))
    world.reset()
    names = list(robot.dof_names)
    if len(names) != 24 or set(names) != EXPECTED_DOF_NAMES:
        raise RuntimeError('24-DOF name contract mismatch: ' + repr(names))
    indices = {n: names.index(n) for n in names}
    order = np.array([indices[n] for n in ROBOT_ORDER])
    ctl = robot.get_articulation_controller()
    ctl.set_gains(kps=np.array([KP_BY_NAME[n] for n in names], dtype=np.float32),
                  kds=np.array([KD_BY_NAME[n] for n in names], dtype=np.float32), save_to_usd=False)
    ctl.set_max_efforts(np.array([EFFORT_LIMIT_BY_NAME[n] for n in names], dtype=np.float32))
    wi = np.array([indices[n] for n in WHEEL_JOINTS], dtype=np.int32)
    robot._articulation_view.set_armatures(np.array([[ARMATURE_BY_NAME[n] for n in WHEEL_JOINTS]], dtype=np.float32), joint_indices=wi)
    for i in wi:
        ctl.switch_dof_control_mode(int(i), 'velocity')
    initial = np.array([DEFAULT_POSITION_BY_NAME.get(n, 0.) for n in names], dtype=np.float32)
    arm = dict(zip(PIPER_JOINTS, policy.config['piper_hold_targets']))
    for n, v in arm.items():
        initial[indices[n]] = v
    robot.set_joint_positions(initial)
    robot.set_joint_velocities(np.zeros(24, dtype=np.float32))
    ctl.apply_action(ArticulationAction(joint_positions=initial))
    set_camera_view(eye=np.array([2.6, 2.6, 1.8]), target=np.array([0., 0., 0.55]), camera_prim_path='/OmniverseKit_Persp')
    trace = []
    start = None
    max_tilt = 0.
    min_height = float('inf')
    max_arm_error = 0.
    yaw_samples = []
    simulation_start_time = world.current_time
    report['dof_names'] = names
    for tick in range(round(args.seconds * 200)):
        if not app.is_running():
            raise RuntimeError('Simulation window closed before the requested duration')
        t = tick * .005
        if tick % 4 == 0:
            pos, quat = robot.get_world_pose()
            q = np.asarray(robot.get_joint_positions())
            dq = np.asarray(robot.get_joint_velocities())
            r = quaternion_to_rotation_matrix(quat)
            tilt = math.acos(float(np.clip(r[2, 2], -1, 1)))
            yaw = math.atan2(float(r[1, 0]), float(r[0, 0]))
            if not np.isfinite(pos).all() or not np.isfinite(q).all() or not np.isfinite(dq).all():
                raise RuntimeError('Non-finite physical state')
            if t >= 2:
                if start is None:
                    start = np.asarray(pos).copy()
                max_tilt = max(max_tilt, tilt)
                yaw_samples.append(yaw)
                min_height = min(min_height, float(pos[2]))
                if pos[2] < .30 or tilt > .65:
                    raise RuntimeError('Simulation stopped: excessive tilt or low base height')
            scale = max(0., min(1., (t - 2.) / 1., (args.seconds - 2. - t) / 1.))
            positions, velocities = policy.step(quat, robot.get_angular_velocity(), q[order], dq[order], command * scale)
            hold = dict(arm)
            if args.arm_demo and 3. <= t < args.seconds - 2.:
                envelope = min(1., t - 3., args.seconds - 2. - t)
                hold['joint1'] = envelope * .15 * math.sin((t - 3.) * math.pi / 2.)
                gap = .025 - envelope * .0075 * (1. - math.cos((t - 3.) * math.pi / 2.))
                hold['joint7'], hold['joint8'] = gap, -gap
            positions.update(hold)
            if t >= 2:
                max_arm_error = max(max_arm_error, max(abs(float(q[indices[n]]) - hold[n]) for n in PIPER_JOINTS[:6]))
            pnames = list(positions)
            ctl.apply_action(ArticulationAction(joint_positions=np.array([positions[n] for n in pnames]),
                                                joint_indices=np.array([indices[n] for n in pnames])))
            ctl.apply_action(ArticulationAction(joint_velocities=np.array([velocities[n] for n in WHEEL_JOINTS]), joint_indices=wi))
            if tick % 20 == 0:
                trace.append({'time_s': t, 'position_m': np.asarray(pos).tolist(), 'tilt_rad': tilt, 'yaw_rad': yaw,
                              'command': (command * scale).tolist(), 'arm_targets': hold,
                              'arm_positions': {n: float(q[indices[n]]) for n in PIPER_JOINTS}})
        world.step(render=(tick % 8 == 0))
    end, _ = robot.get_world_pose()
    report.update(status='COMPLETED', simulated_seconds=world.current_time - simulation_start_time, onnx_calls=policy.calls,
                  minimum_base_height_m=min_height, maximum_tilt_rad=max_tilt, maximum_arm_error_rad=max_arm_error,
                  displacement_after_settle_m=(np.asarray(end) - start).tolist(),
                  yaw_change_after_settle_rad=float(np.unwrap(yaw_samples)[-1] - yaw_samples[0]),
                  final_linear_velocity_mps=np.asarray(robot.get_linear_velocity()).tolist())
    (args.output / 'trace.json').write_text(json.dumps(trace, indent=2) + '\n')
except BaseException as exc:
    report.update(status='FAILED', error=str(exc), onnx_calls=policy.calls)
    if 'trace' in locals():
        (args.output / 'trace.json').write_text(json.dumps(trace, indent=2) + '\n')
    raise
finally:
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report), flush=True)
    app.close()
