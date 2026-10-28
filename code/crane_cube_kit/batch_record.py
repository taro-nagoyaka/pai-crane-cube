"""One isolated automatic teacher episode. Called by collect_batch.py, not manually."""
import argparse
import json
from pathlib import Path
import subprocess
import threading
import time

import numpy as np

from batch_support import PoseTextParser, verify_episode
from common import FPS
from batch_archive import save_trial


class CubePoseTap:
    def __init__(self, error_path):
        self.lock = threading.Lock()
        self.latest = None
        self.lines = 0
        self.error = error_path.open('w')
        self.process = subprocess.Popen(
            ['ign', 'topic', '-e', '-t', '/world/default/pose/info'],
            stdout=subprocess.PIPE, stderr=self.error, text=True, bufsize=1,
            start_new_session=True)
        self.parser = PoseTextParser(self.update)
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()

    def update(self, stamp, xyz):
        with self.lock:
            self.latest = (stamp, xyz.copy(), time.monotonic())

    def read(self):
        for line in self.process.stdout:
            self.lines += 1
            self.parser.feed(line)

    def snapshot(self, sim):
        with self.lock:
            if self.latest is None:
                return None
            stamp, xyz, wall = self.latest
            if not -.05 <= sim - stamp <= .20 or time.monotonic() - wall > 2:
                return None
            return xyz.copy(), stamp

    def close(self):
        from auto_record import stop_child
        stop_child(self.process)
        self.thread.join(timeout=2)
        self.process.stdout.close()
        self.error.close()


def run(node, args, tap, settings):
    from auto_record import stop_child
    from teacher_reference import combined_reference
    deadline = time.monotonic() + 180
    started = time.monotonic()
    next_notice = 0.
    expected_xy = np.array([settings['dx'], -.1 + settings['dy']])
    stable = []
    while True:
        snap = node.snapshot()
        cube = tap.snapshot(snap[0]) if snap is not None else None
        arm_ready, hand_ready = node.arm.server_is_ready(), node.hand.server_is_ready()
        if time.monotonic() >= next_notice:
            with node.lock:
                joint_seen, image_seen = node.q is not None, node.rgb is not None
            with tap.lock:
                raw_stamp = None if tap.latest is None else tap.latest[0]
            now = node.get_clock().now().nanoseconds * 1e-9
            print('待機: %.0f/180秒 関節=%s カメラ=%s 新鮮な観測=%s arm=%s gripper=%s '
                  'cube=%s sim=%.3f cube_stamp=%s pose受信行=%d' % (
                      time.monotonic()-started, joint_seen, image_seen, snap is not None,
                      arm_ready, hand_ready, cube is not None, now, raw_stamp, tap.lines), flush=True)
            next_notice = time.monotonic() + 5
        if node.fault:
            raise RuntimeError(node.fault)
        if tap.process.poll() is not None:
            raise RuntimeError('Gazebo pose reader exited; see pose_reader.log')
        if snap is not None and cube is not None and arm_ready and hand_ready:
            if np.max(np.abs(snap[1][:4])) > .08:
                raise RuntimeError('Fresh trial is not upright; another controller may be commanding it')
            xyz = cube[0]
            if np.linalg.norm(xyz[:2] - expected_xy) > .03 or not .95 < xyz[2] < 1.12:
                raise RuntimeError('Cube pose is not in expected world coordinates; will not label automatically')
            if not stable or cube[1] > stable[-1][0]:
                stable.append((cube[1], xyz))
                stable = [v for v in stable if cube[1] - v[0] <= .5]
                if (len(stable) >= 4 and stable[-1][0] - stable[0][0] >= .3 and
                        max(np.linalg.norm(v[1] - xyz) for v in stable) < .003):
                    initial_cube = xyz.copy()
                    break
        if time.monotonic() > deadline:
            raise RuntimeError('Startup timeout: camera, joint states, controllers, or named cube world pose unavailable')
        time.sleep(.02)

    print('Fresh camera/controllers/cube ready. Starting varied teacher.', flush=True)
    samples, cubes, cube_stamps, stages = [], [], [], []
    issues = []
    teacher = None
    next_tick = None
    last_sim, last_image = snap[0], None
    wall_deadline = time.monotonic() + 900
    complete = False
    infrastructure_error = False
    try:
        teacher = subprocess.Popen(
            ['ros2', 'launch', str(Path(__file__).resolve().parent / 'generated/lift_teacher.launch.py'),
             'use_sim_time:=true'], start_new_session=True)
        while time.monotonic() < wall_deadline:
            if node.fault or node.reference_error:
                raise RuntimeError(node.fault or node.reference_error)
            with node.lock:
                status, recording, refs = node.status, node.recording_started, list(node.references.values())
            if status.startswith('failed:'):
                issues.append(status)
                break
            if status == 'done':
                complete = True
                break
            if teacher.poll() is not None:
                issues.append('teacher_exited_before_done')
                break
            if not recording:
                time.sleep(.005)
                continue
            snap = node.snapshot()
            if snap is None:
                raise RuntimeError('Stale image/joint state during recording')
            sim, q, rgb, js, ims = snap
            if sim < last_sim - .01:
                raise RuntimeError('Simulation clock reset inside a trial')
            last_sim = sim
            if next_tick is None:
                next_tick = sim
            if sim + .002 >= next_tick:
                if sim - next_tick > .035:
                    raise RuntimeError('Missed 10 Hz recording tick; trial discarded')
                cube = tap.snapshot(sim)
                if cube is None:
                    raise RuntimeError('Cube pose became stale; cannot label success')
                if ims == last_image or abs(js - ims) > .1:
                    raise RuntimeError('Duplicated image or image/joint timing mismatch')
                action = combined_reference(q, refs, sim)
                samples.append((sim, q, rgb, action, js, ims))
                cubes.append(cube[0])
                cube_stamps.append(cube[1])
                stages.append(status)
                last_image = ims
                next_tick += 1 / FPS
            time.sleep(.002)
        if not complete and not issues:
            issues.append('teacher_timeout')
        if complete:
            try:
                teacher.wait(timeout=10)
            except subprocess.TimeoutExpired:
                issues.append('teacher_shutdown_timeout')
            if teacher.poll() != 0:
                issues.append('teacher_nonzero_exit')
    except KeyboardInterrupt:
        issues.append('interrupted_partial_episode')
    except Exception as error:
        infrastructure_error = True
        issues.append(str(error))
    finally:
        stop_child(teacher)
    if not samples:
        return dict(passed=False, reasons=issues or ['no_samples'], settings=settings,
                    infrastructure_error=infrastructure_error)
    times = np.array([s[0] for s in samples])
    q = np.stack([s[1] for s in samples]).astype(np.float32)
    actions = np.stack([s[3] for s in samples]).astype(np.float32)
    js, ims = (np.array([s[k] for s in samples]) for k in (4, 5))
    report = verify_episode(times, q, actions, cubes, stages, initial_cube, js, ims)
    report['reasons'].extend(issues)
    report['passed'] = report['passed'] and complete and not issues
    report.update(settings=settings, frames=len(samples))
    if infrastructure_error:
        report['infrastructure_error'] = True
    path = save_trial(args.out, args.result.parent, samples, cubes, cube_stamps, stages,
                      initial_cube, settings, report)
    if path:
        report['episode' if report['passed'] else 'rejected_recording'] = str(path)
    return report


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--settings', type=Path, required=True)
    p.add_argument('--result', type=Path, required=True)
    args = p.parse_args()
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from auto_record import Recorder
    settings = json.loads(args.settings.read_text())
    args.out.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    node = Recorder()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    tap = None
    try:
        tap = CubePoseTap(args.result.parent / 'pose_reader.log')
        report = run(node, args, tap, settings)
    except KeyboardInterrupt:
        report = dict(passed=False, reasons=['interrupted'], settings=settings, infrastructure_error=True)
    except Exception as error:
        report = dict(passed=False, reasons=[str(error)], settings=settings, infrastructure_error=True)
    finally:
        if tap:
            tap.close()
        executor.shutdown()
        thread.join(timeout=5)
        executor.remove_node(node)
        node.destroy_node()
        # SIGINT may have already shut the ROS context down.
        rclpy.try_shutdown()
    args.result.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report), flush=True)
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
