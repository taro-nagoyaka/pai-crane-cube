"""ROS-only terminal teacher / policy client. No LeRobot dependency here."""
import argparse
import base64
import csv
import io
import json
from pathlib import Path
import select
import sys
import termios
import threading
import time
import tty
import urllib.request

import numpy as np
from PIL import Image
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage, JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from common import JOINTS, FPS, IMAGE_SIZE, TASK, bounded_target, image_rgb, reorder


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Robot(Node):
    def __init__(self):
        super().__init__('crane_cube_learning', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.lock = threading.Lock()
        self.q = self.rgb = None
        self.q_time = self.image_time = None
        self.q_wall = self.image_wall = 0.
        self.fault = None
        self.command_id = 0
        self.create_subscription(JointState, '/joint_states', self.on_joint, qos_profile_sensor_data)
        self.create_subscription(RosImage, '/camera/front/image', self.on_image, qos_profile_sensor_data)
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/crane_plus_arm_controller/follow_joint_trajectory')
        self.hand = ActionClient(self, FollowJointTrajectory,
                                 '/crane_plus_gripper_controller/follow_joint_trajectory')

    def on_joint(self, msg):
        try:
            q = reorder(msg.name, msg.position)
            if not np.isfinite(q).all():
                raise ValueError('Nonfinite joint state')
            with self.lock:
                self.q, self.q_time, self.q_wall = q, stamp_s(msg.header.stamp), time.monotonic()
        except (KeyError, ValueError) as e:
            self.fault = str(e)

    def on_image(self, msg):
        try:
            rgb = image_rgb(msg)
            with self.lock:
                self.rgb, self.image_time, self.image_wall = rgb, stamp_s(msg.header.stamp), time.monotonic()
        except ValueError as e:
            self.fault = str(e)

    def snapshot(self):
        with self.lock:
            if self.q is None or self.rgb is None:
                return None
            now = self.get_clock().now().nanoseconds * 1e-9
            age_q, age_image = now - self.q_time, now - self.image_time
            if not (-0.05 <= age_q <= 0.25 and -0.05 <= age_image <= 0.25):
                return None
            # Also detect paused simulation / a dead bridge using wall-clock freshness.
            if time.monotonic() - min(self.q_wall, self.image_wall) > 3.0:
                return None
            return now, self.q.copy(), self.rgb.copy(), self.q_time, self.image_time

    def command(self, q, duration=0.2):
        q = bounded_target(q)
        if not (self.arm.server_is_ready() and self.hand.server_is_ready()):
            raise RuntimeError('Trajectory action servers are not ready')
        self.command_id += 1
        command_id = self.command_id
        for client, names, values in [(self.arm, JOINTS[:4], q[:4]),
                                       (self.hand, JOINTS[4:], q[4:])]:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = names
            point = JointTrajectoryPoint()
            point.positions = values.tolist()
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration % 1) * 1e9)
            goal.trajectory.points = [point]
            future = client.send_goal_async(goal)
            future.add_done_callback(lambda f, c=command_id: self.accepted(f, c))
        return q

    def accepted(self, future, command_id):
        try:
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('Trajectory rejected')
            handle.get_result_async().add_done_callback(
                lambda f, c=command_id: self.completed(f, c))
        except Exception as e:
            self.fault = str(e)

    def completed(self, future, command_id):
        try:
            result = future.result()
            # New targets intentionally replace earlier short trajectories.
            if command_id == self.command_id and result.result.error_code != 0:
                self.fault = f'Trajectory failed: {result.result.error_string}'
        except Exception as e:
            self.fault = str(e)

    def hold(self):
        with self.lock:
            q = None if self.q is None else self.q.copy()
        if q is not None:
            try:
                self.command(q)
            except Exception as e:
                print('Hold failed:', e)


def request(url, path, body):
    payload = json.dumps(body).encode()
    req = urllib.request.Request(url + path, data=payload, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=20) as response:
        return json.load(response)


def save_episode(root, samples, source='manual_target'):
    if len(samples) < FPS * 3:
        raise ValueError('At least 3 seconds of recording are needed')
    stamps = np.array([s[0] for s in samples])
    if not np.allclose(np.diff(stamps), 1 / FPS, atol=0.035):
        raise ValueError('Missing / irregular samples; discard and record again')
    root.mkdir(parents=True, exist_ok=True)
    path = root / time.strftime('episode_%Y%m%d_%H%M%S.npz')
    if path.exists():
        raise FileExistsError(path)
    np.savez_compressed(path,
        state=np.stack([s[1] for s in samples]).astype(np.float32),
        image=np.stack([s[2] for s in samples]),
        action=np.stack([s[3] for s in samples]).astype(np.float32),
        time=stamps - stamps[0],
        joint_stamp=np.array([s[4] for s in samples]),
        image_stamp=np.array([s[5] for s in samples]),
        joint_names=np.array(JOINTS), fps=np.array(FPS),
        success=np.array(True), task=np.array(TASK), source=np.array(source))
    print('\r\nSaved (operator marked successful):', path, '\r')


def run(node, args):
    print('Waiting for fresh joint state + camera + trajectory servers (30 s)...')
    deadline = time.monotonic() + 30
    while node.snapshot() is None or not (node.arm.server_is_ready() and node.hand.server_is_ready()):
        if node.fault:
            raise RuntimeError(node.fault)
        if time.monotonic() > deadline:
            raise RuntimeError('Not ready. Check /joint_states, /camera/front/image, Gazebo Play, action servers.')
        time.sleep(0.05)
    snap = node.snapshot()
    target = snap[1].astype(np.float64)
    selected, step = 0, 0.05
    recording, samples, invalid = False, [], False
    next_tick = snap[0]
    last_time = snap[0]
    last_print = 0.
    last_image_stamp = None
    if args.mode == 'deploy':
        request(args.url, '/reset', {})
        print('Policy mode. q / space stops and holds. Max duration:', args.seconds, 'sim seconds.')
    else:
        print('1..5: select joint; +/-: jog; f: 0.01/0.05 rad; r: record;')
        print('s: save SUCCESS after visually checking cube + 2 s hold; d: discard;')
        print('v: save camera preview; SPACE: hold + discard; q: quit.')
        print('Direct joint control has no MoveIt collision planning. Use small increments.')
    started = snap[0]
    last_deploy_log = started - 1.0
    trace_file = None
    trace_writer = None
    if args.mode == 'deploy' and args.trace:
        trace_path = Path(args.trace)
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        trace_file = trace_path.open('x', newline='', encoding='utf-8')
        trace_writer = csv.writer(trace_file)
        trace_writer.writerow(['time_s', 'inference_wall_s', 'decision', 'max_target_jump_rad',
                               'policy_step', 'n_action_steps', 'temporal_ensemble_coeff'] + [
            prefix + name for prefix in ('state_', 'previous_target_', 'prediction_', 'target_') for name in JOINTS])
        trace_file.flush()
        print('Deployment trace:', trace_path)
    saved_terminal = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = sys.stdin.read(1) if select.select([sys.stdin], [], [], 0.01)[0] else ''
            if key in ('q', '\x03'):
                break
            if key == ' ':
                node.hold()
                recording = False
                samples = []
                if args.mode == 'deploy':
                    break
                with node.lock:
                    target = node.q.astype(np.float64).copy()
                print('\r\nHeld; episode discarded.\r')
            if node.fault:
                raise RuntimeError(node.fault)
            snap = node.snapshot()
            if snap is None:
                # Never bridge a gap with fabricated / duplicated training frames.
                if recording:
                    invalid = True
                if args.mode == 'deploy':
                    raise RuntimeError('Observation stale or simulation paused; stopping')
                continue
            sim_time, state, rgb, q_stamp, i_stamp = snap
            if sim_time < last_time - 0.01:
                raise RuntimeError('Simulation time reset. Restart this program after a scene reset.')
            last_time = sim_time
            if args.mode == 'teach':
                if key in '12345' and key:
                    selected = int(key) - 1
                elif key == 'f':
                    step = 0.01 if step == 0.05 else 0.05
                elif key in ('+', '=', '-'):
                    target[selected] += step * (-1 if key == '-' else 1)
                    target = node.command(target)
                elif key == 'r' and not recording:
                    # Start from a known held target rather than a stale previous command.
                    target = node.command(state)
                    recording, samples, invalid = True, [], False
                    next_tick, last_image_stamp = sim_time, None
                    print('\r\nRecording. Perform grasp, lift, then HOLD for 2 sim seconds.\r')
                elif key == 'd':
                    recording, samples = False, []
                    print('\r\nDiscarded.\r')
                elif key == 's' and recording:
                    recording = False
                    if invalid:
                        print('\r\nNOT saved: time gap / stale data. Reset scene and repeat.\r')
                    else:
                        try:
                            save_episode(Path(args.out), samples)
                        except ValueError as e:
                            print('\r\nNOT saved:', e, '\r')
                    samples = []
                elif key == 'v':
                    Image.fromarray(rgb).save('camera_preview.png')
                    print('\r\nSaved camera_preview.png\r')
            if sim_time + 0.002 >= next_tick:
                if sim_time - next_tick > 0.035:
                    if recording:
                        invalid = True
                    if args.mode == 'deploy':
                        raise RuntimeError('CPU loop missed 10 Hz in simulation time. Lower --rtf and restart.')
                    next_tick = sim_time
                if recording:
                    if i_stamp == last_image_stamp:
                        invalid = True
                    if abs(q_stamp - i_stamp) > 0.1:
                        invalid = True
                    samples.append((sim_time, state, rgb, target.copy(), q_stamp, i_stamp))
                    last_image_stamp = i_stamp
                if args.mode == 'deploy':
                    bio = io.BytesIO()
                    Image.fromarray(rgb).save(bio, format='PNG')
                    wall_start = time.monotonic()
                    result = request(args.url, '/predict', {
                        'state': state.tolist(),
                        'image': base64.b64encode(bio.getvalue()).decode()})
                    inference_wall = time.monotonic() - wall_start
                    inference_end = node.get_clock().now().nanoseconds * 1e-9
                    desired = bounded_target(result['action'])
                    jumps = np.abs(desired - target)
                    max_jump = float(jumps.max())
                    decision = 'allowed'
                    if inference_end - sim_time > 1 / FPS:
                        decision = 'rejected_latency'
                    elif max_jump > args.max_step:
                        decision = 'rejected_jump'
                    # Log ALL finite predictions BEFORE rejection or command submission.
                    # target_ is the proposed bounded target; it is not proof of execution.
                    if trace_writer is not None:
                        trace_writer.writerow([
                            sim_time - started, inference_wall, decision, max_jump,
                            result.get('policy_step', ''), result.get('n_action_steps', ''),
                            result.get('temporal_ensemble_coeff', ''),
                            *state.tolist(), *target.tolist(), *result['action'], *desired.tolist()])
                        trace_file.flush()
                    if decision == 'rejected_latency':
                        raise RuntimeError('Inference exceeded 0.1 sim seconds. Lower --rtf; do not use stale actions.')
                    if decision == 'rejected_jump':
                        j = int(np.argmax(jumps))
                        raise RuntimeError(
                            f'Policy target jump too large at sim={sim_time-started:.3f}s: '
                            f'{JOINTS[j]}, previous={target[j]:.4f}, proposed={desired[j]:.4f}, '
                            f'jump={max_jump:.4f} rad > {args.max_step:.4f}. '
                            'Rejected prediction logged; limits unchanged.')
                    target = node.command(desired)
                    if sim_time - last_deploy_log >= 1.0:
                        last_deploy_log = sim_time
                        print('\nsim=%.1fs q=%s target=%s max_error=%.3f rad' % (
                            sim_time - started, np.round(state, 3), np.round(target, 3),
                            np.max(np.abs(target - state))), flush=True)
                    if sim_time - started >= args.seconds:
                        break
                next_tick += 1 / FPS
            if args.mode == 'teach' and time.monotonic() - last_print > 0.5:
                last_print = time.monotonic()
                print('\rjoint=%d step=%.2f q=%s REC=%s frames=%d invalid=%s   ' % (
                    selected + 1, step, np.round(state, 3), recording, len(samples), invalid), end='', flush=True)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, saved_terminal)
        node.hold()
        time.sleep(0.25)
        if trace_file is not None:
            trace_file.close()
            print('\nStopped. Deployment trace saved:', args.trace)
        elif args.mode == 'deploy':
            print('\nStopped (policy execution).')
        else:
            print('\nStopped. Unsaved recording discarded.')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('mode', choices=['teach', 'deploy'])
    p.add_argument('--out', default='raw_episodes')
    p.add_argument('--url', default='http://127.0.0.1:8766')
    p.add_argument('--seconds', type=float, default=60)
    p.add_argument('--max-step', type=float, default=0.15)
    p.add_argument('--trace', default=None, help='Write deploy observations/predictions/commands to a new CSV')
    args = p.parse_args()
    if not sys.stdin.isatty():
        p.error('Run in an interactive terminal (docker exec -it ...)')
    rclpy.init(args=[])
    node = Robot()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        run(node, args)
    finally:
        node.hold()
        time.sleep(0.2)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
