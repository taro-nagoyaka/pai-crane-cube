"""Compare ACT against hold-current-state and training-mean baselines.

Run in the existing LeRobot venv:
python diagnose_model.py --checkpoint outputs/act_auto_2000/checkpoints/last/pretrained_model --data data_auto
No new training; existing datasets/checkpoints are read only.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from common import JOINTS
from policy_server import load_policy, predict


def metric(prediction, target, mask=None):
    prediction = np.asarray(prediction, dtype=float)
    target = np.asarray(target, dtype=float)
    if prediction.shape != target.shape or prediction.ndim != 2 or prediction.shape[1] != 5:
        raise ValueError('Expected matching [frames, 5] arrays')
    error = np.abs(prediction - target)
    if mask is not None:
        error = error[np.asarray(mask, dtype=bool)]
    if len(error) == 0:
        return None
    return {'frames': len(error), 'mae_rad': float(error.mean()),
            'per_joint_rad': dict(zip(JOINTS, error.mean(axis=0).tolist()))}


def report_arrays(states, actions, predictions, episodes, mean_action):
    states, actions, predictions = map(np.asarray, (states, actions, predictions))
    # Moving is defined by a change in the observed robot position, not by model output.
    moving = np.zeros(len(states), dtype=bool)
    for i in range(1, len(states)):
        if episodes[i] == episodes[i - 1]:
            moving[i] = np.max(np.abs(states[i] - states[i - 1])) > 0.005
    mean_prediction = np.broadcast_to(np.asarray(mean_action), actions.shape)
    return {'frames': len(states), 'episodes': len(set(episodes)),
            'model': metric(predictions, actions),
            'hold_current_angle': metric(states, actions),
            'training_mean_action': metric(mean_prediction, actions),
            'moving_model': metric(predictions, actions, moving),
            'moving_hold_current_angle': metric(states, actions, moving),
            'moving_definition': 'same episode and max observed joint change > 0.005 rad per sampled frame',
            'target_action_range_rad': dict(zip(JOINTS, np.ptp(actions, axis=0).tolist()))}


def evaluate_split(ds, policy, pre, post, mean_action, output, limit):
    count = len(ds) if limit <= 0 else min(len(ds), limit)
    states, actions, predictions, episodes = [], [], [], []
    previous = None
    columns = ['episode', 'frame_index', 'time_s']
    columns += [prefix + j for prefix in ('state_', 'target_', 'prediction_') for j in JOINTS]
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        for i in range(count):
            frame = ds[i]
            episode = int(frame['episode_index'])
            if episode != previous:
                policy.reset()
                previous = episode
            q = frame['observation.state'].numpy()
            target = frame['action'].numpy()
            rgb = (frame['observation.images.front'].permute(1, 2, 0).numpy() * 255).round().clip(0, 255).astype(np.uint8)
            action = predict(policy, pre, post, q, rgb)
            states.append(q.copy())
            actions.append(target.copy())
            predictions.append(action.copy())
            episodes.append(episode)
            writer.writerow([episode, int(frame['frame_index']), float(frame['timestamp']),
                             *q.tolist(), *target.tolist(), *action.tolist()])
            if (i + 1) % 100 == 0:
                print(f'{output.stem}: {i + 1}/{count}', flush=True)
    if not states:
        raise ValueError('Dataset contains no frames')
    return report_arrays(states, actions, predictions, episodes, mean_action)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--checkpoint', required=True)
    p.add_argument('--data', default='data_auto')
    p.add_argument('--out', default='diagnostics_2000')
    p.add_argument('--max-frames', type=int, default=0, help='0 = all frames')
    args = p.parse_args()
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    data = Path(args.data).resolve()
    for name in ('train', 'validation'):
        if not (data / name / 'meta/info.json').is_file():
            p.error(f'No local dataset at {data / name}')
    output = Path(args.out)
    output.mkdir(parents=True, exist_ok=False)
    train = LeRobotDataset('local/crane_cube_train', root=data / 'train')
    mean_action = np.asarray(train.meta.stats['action']['mean'], dtype=np.float32)
    if mean_action.shape != (5,):
        raise ValueError('Unexpected training action statistics')
    policy, pre, post = load_policy(args.checkpoint, 4)
    report = {'checkpoint': str(Path(args.checkpoint).resolve()),
              'data': str(data), 'units': 'radians',
              'limitation': 'Teacher-forced action prediction, NOT closed-loop grasp success. '
                            'Hold-current-angle baseline alone cannot accomplish lifting.',
              'mean_baseline_source': 'TRAINING action statistics only; no validation statistics used'}
    for name, ds in [('train', train), ('validation', LeRobotDataset(
            'local/crane_cube_validation', root=data / 'validation'))]:
        policy.reset()
        report[name] = evaluate_split(ds, policy, pre, post, mean_action,
                                     output / f'{name}_trace.csv', args.max_frames)
    (output / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print('\n--- Diagnostic summary: MAE in radians ---')
    for name in ('train', 'validation'):
        result = report[name]
        print(name, 'frames=', result['frames'], 'episodes=', result['episodes'])
        for key in ('model', 'hold_current_angle', 'training_mean_action',
                    'moving_model', 'moving_hold_current_angle'):
            item = result[key]
            print(f'  {key}: ' + ('no moving frames' if item is None else f"{item['mae_rad']:.6f} ({item['frames']} frames)"))
    print('Saved:', output / 'report.json')
    print('Per-frame traces:', output / 'train_trace.csv', output / 'validation_trace.csv')
    print('Hold baseline is a diagnostic; it does not mean holding still solves the task.')


if __name__ == '__main__':
    main()
