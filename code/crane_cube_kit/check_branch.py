"""Check new teacher recordings without LeRobot or ROS imports."""
import argparse
from pathlib import Path
import numpy as np
from common import JOINTS


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--raw', default='raw_auto_branch')
    args = p.parse_args()
    paths = sorted(Path(args.raw).glob('episode_*.npz'))
    if not paths:
        p.error('No episodes found')
    failed = []
    for path in paths:
        with np.load(path, allow_pickle=False) as episode:
            if episode['joint_names'].tolist() != JOINTS:
                raise ValueError(f'{path}: joint order differs')
            states, actions = episode['state'], episode['action']
            if not np.isfinite(states).all() or not np.isfinite(actions).all():
                raise ValueError(f'{path}: nonfinite values')
            # Allow small physical tracking / timestamp differences around 0.02.
            outside = np.any(states[:, 1:3] > 0.03, axis=1)
            outside_action = np.any(actions[:, 1:3] > 0.025, axis=1)
            status = 'PASS' if not outside.any() and not outside_action.any() else 'CHECK'
            if status != 'PASS':
                failed.append(path.name)
            print(path.name, status, 'frames=', len(states),
                  'q2_range=', tuple(np.round([states[:, 1].min(), states[:, 1].max()], 4)),
                  'q3_range=', tuple(np.round([states[:, 2].min(), states[:, 2].max()], 4)),
                  'outside_state_frames=', int(outside.sum()),
                  'outside_action_frames=', int(outside_action.sum()))
    print('Branch check only; physical grasp/lift success still needs visual verification.')
    if failed:
        raise SystemExit('Unexpected branch: ' + ', '.join(failed))


if __name__ == '__main__':
    main()
