"""Diagnostic replay of recorded ACTIONS through the policy's trajectory interface.

This is a control-interface diagnostic, not learned-policy deployment.
Requires only the ROS/system Python environment, not the LeRobot venv.
"""
import argparse
import csv
from pathlib import Path
import select
import sys
import termios
import threading
import time
import tty

import numpy as np
from common import FPS, JOINTS, LOWER, UPPER


def load_episode(path):
    with np.load(path, allow_pickle=False) as episode:
        if episode['joint_names'].tolist() != JOINTS or int(episode['fps']) != FPS:
            raise ValueError('Episode joint order / FPS differs')
        if not bool(episode['success']):
            raise ValueError('Episode was not marked successful')
        actions = episode['action'].astype(np.float64)
        initial_state = episode['state'][0].astype(np.float64)
    if actions.ndim != 2 or actions.shape[1] != 5 or len(actions) < 2:
        raise ValueError('Expected [frames, 5] actions')
    if initial_state.shape != (5,):
        raise ValueError('Expected five initial joint positions')
    if not np.isfinite(actions).all() or not np.isfinite(initial_state).all():
        raise ValueError('Nonfinite recorded values')
    if np.any(actions < LOWER - 1e-5) or np.any(actions > UPPER + 1e-5):
        raise ValueError('Recorded actions outside joint limits')
    return initial_state, actions


def replay(node, actions, initial, trace_path, max_step):
    print('診断用: 保存済みの教師指令を、モデルと同じ0.2秒の軌道指令で再生します。')
    print('キューブとアームは初期位置が必要です。q / Spaceで中断できます。')
    deadline = time.monotonic() + 30
    while node.snapshot() is None or not (node.arm.server_is_ready() and node.hand.server_is_ready()):
        if node.fault:
            raise RuntimeError(node.fault)
        if time.monotonic() > deadline:
            raise RuntimeError('カメラ付きGazebo・関節状態・コントローラーを確認してください。')
        time.sleep(0.05)
    snapshot = node.snapshot()
    if np.max(np.abs(snapshot[1] - initial)) > 0.08:
        raise RuntimeError('初期姿勢が教師データと異なります。Gazeboを終了・再起動してください。')
    if np.max(np.abs(actions[0] - snapshot[1])) > max_step or np.max(np.abs(np.diff(actions, axis=0))) > max_step:
        raise RuntimeError('教師指令の変化が0.15 rad上限を超えます。送信せず停止します。')
    start = snapshot[0]
    last_time = start
    previous = snapshot[1].copy()
    completed = False
    terminal = termios.tcgetattr(sys.stdin)
    trace_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with trace_path.open('x', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time_s', 'frame_index', 'max_tracking_error_rad'] + [
                prefix + name for prefix in ('state_', 'target_') for name in JOINTS])
            tty.setcbreak(sys.stdin.fileno())
            for index, action in enumerate(actions):
                scheduled = start + index / FPS
                while True:
                    if select.select([sys.stdin], [], [], 0)[0] and sys.stdin.read(1) in ('q', ' ', '\x03'):
                        raise KeyboardInterrupt
                    if node.fault:
                        raise RuntimeError(node.fault)
                    snapshot = node.snapshot()
                    if snapshot is None:
                        raise RuntimeError('観測が古いかGazeboが停止しています。')
                    sim, state = snapshot[:2]
                    if sim < last_time - 0.01:
                        raise RuntimeError('シミュレーション時刻がリセットされました。')
                    last_time = sim
                    if sim + 0.002 >= scheduled:
                        break
                    time.sleep(0.002)
                if sim - scheduled > 0.035:
                    raise RuntimeError('10 Hzの再生に遅れが出ました。')
                if np.max(np.abs(action - previous)) > max_step:
                    raise RuntimeError('指令の飛びを検出したため停止しました。')
                node.command(action, duration=0.2)
                previous = action.copy()
                tracking = float(np.max(np.abs(action - state)))
                writer.writerow([sim - start, index, tracking, *state.tolist(), *action.tolist()])
                stream.flush()
                if index % FPS == 0:
                    print('\rsim=%.1fs q=%s target=%s tracking=%.3f rad' % (
                        sim - start, np.round(state, 3), np.round(action, 3), tracking), flush=True)
            # Allow the final trajectory to finish; do not release or overwrite the gripper target.
            settle_start = node.get_clock().now().nanoseconds * 1e-9
            wall_deadline = time.monotonic() + 15
            while node.get_clock().now().nanoseconds * 1e-9 - settle_start < 0.5:
                if node.fault:
                    raise RuntimeError(node.fault)
                if time.monotonic() > wall_deadline:
                    raise RuntimeError('Final settling timed out; simulation clock may be paused')
                time.sleep(0.01)
            completed = True
            print('\n教師指令の再生終了。これは学習モデルの成功ではなく、制御接続の診断です。')
            print('キューブを掴んで持ち上げられたか、最後まで保持できたか確認してください。')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal)
        if not completed:
            node.hold()
            time.sleep(0.25)
        print('診断ログ:', trace_path)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--raw', default='raw_auto_branch')
    p.add_argument('--episode', default=None, help='Optional explicit episode .npz')
    p.add_argument('--trace', default='replay_teacher_trace.csv')
    p.add_argument('--max-step', type=float, default=0.15)
    args = p.parse_args()
    if not sys.stdin.isatty():
        p.error('対話ターミナルで実行してください。')
    if args.episode:
        episode = Path(args.episode)
    else:
        paths = sorted(Path(args.raw).glob('episode_*.npz'))
        if not paths:
            p.error('成功記録が見つかりません。--rawまたは--episodeを確認してください。')
        episode = paths[0]
    initial, actions = load_episode(episode)
    print('教師データ:', episode, 'frames=', len(actions), 'seconds=', len(actions) / FPS)
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from robot import Robot
    rclpy.init(args=[])
    node = Robot()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        replay(node, actions, initial, Path(args.trace), args.max_step)
    except KeyboardInterrupt:
        print('\n診断再生を中断しました。')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
