"""Restart the complete simulation per attempt; collect only verified varied episodes."""
import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from batch_support import make_trial, trial_environment, trial_settings


def stop(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in [(signal.SIGINT, 15), (signal.SIGTERM, 10), (signal.SIGKILL, 5)]:
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass
        except ProcessLookupError:
            return


def count_success(out):
    # Only verified batch episodes count toward the resumable target.
    import numpy as np
    count = 0
    for path in out.glob('episode_*.npz'):
        with np.load(path, allow_pickle=False) as ep:
            if 'automatic_verification' not in ep or not bool(ep['success']):
                raise ValueError('Use a separate batch folder; found non-batch data: ' + str(path))
            if not json.loads(str(ep['automatic_verification']))['passed']:
                raise ValueError('Unverified batch data: ' + str(path))
            count += 1
    return count


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, default=Path('raw_auto_varied'))
    p.add_argument('--episodes', type=int, default=24, help='Target successful TOTAL in --out; rerun to resume')
    p.add_argument('--max-attempts', type=int, default=48, help='Maximum attempts in THIS invocation')
    p.add_argument('--seed', type=int, default=25)
    p.add_argument('--domain-id', type=int, default=82, help='Dedicated ROS domain for this batch')
    args = p.parse_args()
    if args.episodes < 1 or args.max_attempts < 1 or args.seed < 0 or not 0 <= args.domain_id <= 101:
        p.error('episodes/max-attempts must be positive; seed >= 0; domain-id 0..101')
    root = Path(__file__).resolve().parent
    args.out = args.out.resolve()
    args.out.mkdir(parents=True, exist_ok=True)
    lock = (args.out / '.batch.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        p.error('Another batch is using this output folder')
    logs = args.out / 'batch_logs'
    logs.mkdir(exist_ok=True)
    existing_trials = list(logs.glob('trial_*'))
    index = max([int(v.name.split('_')[1]) for v in existing_trials], default=-1) + 1
    successes = count_success(args.out)
    infra_failures = 0
    print(f'自動収集（判定v2・除外データ保管あり）: 保存済み {successes} / 目標 {args.episodes} 件。Ctrl+Cで中断。', flush=True)
    print('既存Gazeboとモデルサーバーは終了してください。この処理が専用Gazeboを起動します。', flush=True)
    try:
        for attempt in range(args.max_attempts):
            if successes >= args.episodes:
                break
            settings = trial_settings(args.seed, index)
            directory = logs / f'trial_{index:05d}'
            directory.mkdir(exist_ok=False)
            settings_path, result_path = directory / 'settings.json', directory / 'result.json'
            settings_path.write_text(json.dumps(settings, indent=2), encoding='utf-8')
            launch = directory / 'trial.launch.py'
            make_trial(root / 'generated/table_camera.sdf', root / 'generated/cube_camera.launch.py',
                       directory / 'trial.sdf', launch, settings)
            env = os.environ.copy()
            env.update(trial_environment(settings))
            env['ROS_DOMAIN_ID'] = str(args.domain_id)
            env['IGN_PARTITION'] = f'crane_batch_{os.getpid()}_{index}'
            # Use the container's normal DDS interface. Forcing loopback can disable
            # multicast discovery under CycloneDDS. Domain 82 already isolates ROS.
            sim = record = None
            print(f'試行 {index + 1}: {settings["profile"]} / 保存済み {successes}', flush=True)
            try:
                with (directory / 'gazebo.log').open('w') as sim_log, (directory / 'record.log').open('w') as rec_log:
                    sim = subprocess.Popen(['ros2', 'launch', str(launch)], env=env,
                                           stdout=sim_log, stderr=subprocess.STDOUT, start_new_session=True)
                    record = subprocess.Popen([sys.executable, '-u', str(root / 'batch_record.py'),
                        '--out', str(args.out), '--settings', str(settings_path), '--result', str(result_path)],
                        env=env, stdout=rec_log, stderr=subprocess.STDOUT, start_new_session=True)
                    deadline = time.monotonic() + 1200
                    with (directory / 'record.log').open() as progress:
                        while record.poll() is None:
                            for line in progress:
                                if line.startswith(('待機:', '準備完了:', 'Teacher:', 'Fresh ')):
                                    print(line.rstrip(), flush=True)
                            if sim.poll() is not None:
                                raise RuntimeError('Gazebo launch exited. See ' + str(directory / 'gazebo.log'))
                            if time.monotonic() > deadline:
                                raise RuntimeError('Attempt wall-clock timeout')
                            time.sleep(.25)
                if not result_path.exists():
                    raise RuntimeError('Recorder produced no result. See ' + str(directory / 'record.log'))
                report = json.loads(result_path.read_text())
            except Exception as error:
                report = dict(passed=False, infrastructure_error=True, reasons=[str(error)], settings=settings)
                result_path.write_text(json.dumps(report, indent=2), encoding='utf-8')
            finally:
                stop(record)
                stop(sim)
            if report['passed']:
                successes += 1
                infra_failures = 0
                print(f'保存成功 {successes}/{args.episodes}: {Path(report["episode"]).name}', flush=True)
            else:
                infra_failures = infra_failures + 1 if report.get('infrastructure_error') else 0
                print('保存しません: ' + ', '.join(report['reasons']), flush=True)
                if report.get('rejected_recording'):
                    print('除外データは学習用と別に保管: ' + report['rejected_recording'], flush=True)
                print('詳細: ' + str(directory), flush=True)
            if infra_failures >= 3:
                print('起動・記録のエラーが3回続いたため停止します。ログを確認してください。', flush=True)
                break
            index += 1
    except KeyboardInterrupt:
        print('\n中断しました。保存済みエピソードは残ります。', flush=True)
    finally:
        lock.close()
    print(f'終了: 成功データ {successes}/{args.episodes} 件。出力先 {args.out}', flush=True)
    return 0 if successes >= args.episodes else 2


if __name__ == '__main__':
    raise SystemExit(main())
