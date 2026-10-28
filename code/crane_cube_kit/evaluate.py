"""Held-out episode teacher-forced action MAE; NOT a grasp success rate."""
import argparse
import json
from pathlib import Path
import numpy as np
from common import JOINTS
from policy_server import load_policy, predict


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--checkpoint', required=True)
    p.add_argument('--data', default='data/validation')
    p.add_argument('--max-frames', type=int, default=500)
    args = p.parse_args()
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset('local/crane_cube_validation', root=Path(args.data).resolve())
    policy, pre, post = load_policy(args.checkpoint, 4)
    errors, previous = [], None
    for i in range(min(len(ds), args.max_frames)):
        frame = ds[i]
        episode = int(frame['episode_index'])
        if episode != previous:
            policy.reset()
            previous = episode
        rgb = (frame['observation.images.front'].permute(1, 2, 0).numpy() * 255).round().clip(0, 255).astype(np.uint8)
        action = predict(policy, pre, post, frame['observation.state'].numpy(), rgb)
        errors.append(np.abs(action - frame['action'].numpy()))
    if not errors:
        raise ValueError('No validation frames')
    report = {'frames': len(errors), 'metric': 'held-out teacher-forced absolute action MAE (radians)',
              'per_joint': dict(zip(JOINTS, np.mean(errors, axis=0).tolist())),
              'overall': float(np.mean(errors)),
              'limitation': 'Does not measure closed-loop grasp/lift success in Gazebo.'}
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
