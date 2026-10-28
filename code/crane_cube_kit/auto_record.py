"""Start automatic MoveIt teacher and record; only final success labeling is manual."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from common import FPS, LOWER, UPPER
from robot import Robot, save_episode, stamp_s
from teacher_reference import combined_reference


class Recorder(Robot):
    def __init__(self):
        super().__init__()
        self.status = 'waiting'
        self.recording_started = False
        self.references = {}
        self.reference_error = None
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/crane_cube_teacher/status', self.on_status, qos)
        self.create_subscription(JointTrajectory, '/crane_cube_teacher/reference', self.on_reference, qos)

    def on_status(self, msg):
        with self.lock:
            if msg.data == 'recording':
                self.references.clear()
                self.recording_started = True
            self.status = msg.data
        print('\nTeacher:', msg.data, flush=True)

    def on_reference(self, msg):
        try:
            reference = (list(msg.joint_names), stamp_s(msg.header.stamp),
                         [stamp_s(p.time_from_start) for p in msg.points],
                         [list(p.positions) for p in msg.points])
            # Validate immediately so executor-thread exceptions cannot go unnoticed.
            combined_reference(np.zeros(5), [reference], reference[1])
            with self.lock:
                self.references[tuple(msg.joint_names)] = reference
        except (ValueError, IndexError) as e:
            self.reference_error = str(e)


def stop_child(child):
    if child is None or child.poll() is not None:
        return
    try:
        os.killpg(child.pid, signal.SIGINT)
        child.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid, signal.SIGTERM)
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()
    except ProcessLookupError:
        pass


def collect(node, args):
    print('カメラと関節状態を確認しています…', flush=True)
    deadline = time.monotonic() + 30
    while node.snapshot() is None:
        if node.fault:
            raise RuntimeError(node.fault)
        if time.monotonic() > deadline:
            raise RuntimeError('カメラ付きGazeboが必要です。元のcube_camera.launch.pyを起動してください。')
        time.sleep(0.05)
    root = Path(__file__).resolve().parent
    samples, invalid = [], []
    next_tick = None
    previous_image = None
    previous_time = node.snapshot()[0]
    child = None
    completed = False
    waiting_deadline = time.monotonic() + 120
    total_deadline = time.monotonic() + args.timeout
    last_notice = 0
    try:
        print('自動で「接近 → 把持 → 持ち上げ → 直立 → 2秒保持」を実行・記録します。', flush=True)
        child = subprocess.Popen(['ros2', 'launch', str(root / 'generated/lift_teacher.launch.py'),
                                  'use_sim_time:=true'], start_new_session=True)
        while time.monotonic() < total_deadline:
            if node.fault or node.reference_error:
                raise RuntimeError(node.fault or node.reference_error)
            with node.lock:
                status = node.status
                recording = node.recording_started
                refs = list(node.references.values())
            if status.startswith('failed:'):
                raise RuntimeError(status)
            if status == 'done':
                completed = True
                break
            if child.poll() is not None:
                raise RuntimeError(f'Teacher ended before completion (exit {child.returncode}). See launch log.')
            if not recording:
                if time.monotonic() > waiting_deadline:
                    raise RuntimeError('Teacher startup timed out. Check MoveIt / teacher launch log.')
                time.sleep(0.01)
                continue
            snap = node.snapshot()
            if snap is None:
                if not invalid:
                    invalid.append('古い観測・画像欠落・停止中のシミュレーション')
                time.sleep(0.005)
                continue
            sim, state, image, q_stamp, i_stamp = snap
            if sim < previous_time - 0.01:
                raise RuntimeError('Simulation clock reset. Restart collector after scene restart.')
            previous_time = sim
            if next_tick is None:
                next_tick = sim
            if sim + 0.002 >= next_tick:
                if sim - next_tick > 0.035:
                    if '10Hz記録の欠落' not in invalid:
                        invalid.append('10Hz記録の欠落')
                    next_tick = sim
                if previous_image == i_stamp and '画像重複' not in invalid:
                    invalid.append('画像重複')
                if abs(q_stamp - i_stamp) > 0.1 and '観測時刻のずれ' not in invalid:
                    invalid.append('観測時刻のずれ')
                action = combined_reference(state, refs, sim)
                if np.any(action < LOWER - 1e-5) or np.any(action > UPPER + 1e-5):
                    raise RuntimeError('Teacher reference exceeds known joint limits')
                samples.append((sim, state, image, action, q_stamp, i_stamp))
                previous_image = i_stamp
                next_tick += 1 / FPS
            if time.monotonic() - last_notice > 5:
                print(f'記録中: {len(samples)}フレーム / {status}', flush=True)
                last_notice = time.monotonic()
            time.sleep(0.002)
        if not completed:
            raise RuntimeError('Teacher timed out. No data saved.')
        # No controller commands from this observer on successful completion.
        # Gazebo's controllers continue holding the teacher's final target.
        try:
            child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            stop_child(child)
        if child.returncode != 0:
            raise RuntimeError(f'Teacher launch returned {child.returncode}; data not saved')
        if invalid:
            raise RuntimeError('記録品質に問題があるため保存しません: ' + ', '.join(invalid))
        print('\n自動動作が終了しました。学習済みモデルの実行ではなく、教師データの作成です。')
        answer = input('キューブを掴んだまま上まで持ち上げ、2秒保持できましたか？ [y/N]: ').strip().lower()
        if answer == 'y':
            save_episode(Path(args.out), samples, source='moveit_plan_reference_linear_lookahead_0.2s')
        else:
            print('失敗データとして破棄しました。')
    finally:
        stop_child(child)
        if not completed:
            node.hold()
            time.sleep(0.25)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', default='raw_auto_episodes')
    p.add_argument('--timeout', type=float, default=900)
    args = p.parse_args()
    rclpy.init(args=[])
    node = Recorder()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        collect(node, args)
    except KeyboardInterrupt:
        print('\n中断しました。未保存の記録は破棄します。')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
