"""Run in the isolated LeRobot venv. No ROS imports."""
import argparse
import json
from pathlib import Path
import random
import numpy as np
from common import FPS, IMAGE_SIZE, JOINTS, TASK, LOWER, UPPER


def validate(path):
    with np.load(path, allow_pickle=False) as ep:
        n = len(ep['state'])
        if int(ep['fps']) != FPS or ep['joint_names'].tolist() != JOINTS:
            raise ValueError(f'{path}: schema / joint order mismatch')
        if not bool(ep['success']):
            raise ValueError(f'{path}: not marked successful')
        for name in ('state', 'action'):
            a = ep[name]
            if a.shape != (n, 5) or not np.isfinite(a).all():
                raise ValueError(f'{path}: invalid {name}')
            if np.any(a < LOWER - 0.03) or np.any(a > UPPER + 0.03):
                raise ValueError(f'{path}: {name} outside joint limits')
        if ep['image'].shape != (n, IMAGE_SIZE, IMAGE_SIZE, 3) or ep['image'].dtype != np.uint8:
            raise ValueError(f'{path}: invalid image array')
        if n < FPS * 3 or not np.allclose(np.diff(ep['time']), 1 / FPS, atol=0.035):
            raise ValueError(f'{path}: episode too short or irregular timestamps')
    return n


def convert(files, output, repo_id):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    features = {
        'observation.state': {'dtype': 'float32', 'shape': (5,), 'names': JOINTS},
        'action': {'dtype': 'float32', 'shape': (5,), 'names': JOINTS},
        'observation.images.front': {'dtype': 'image', 'shape': (IMAGE_SIZE, IMAGE_SIZE, 3),
                                     'names': ['height', 'width', 'channels']},
    }
    dataset = LeRobotDataset.create(repo_id=repo_id, root=output, fps=FPS,
                                    robot_type='crane_plus_gazebo', features=features,
                                    use_videos=False)
    try:
        for path in files:
            with np.load(path, allow_pickle=False) as ep:
                for i in range(len(ep['state'])):
                    dataset.add_frame({'observation.state': ep['state'][i].copy(),
                                       'observation.images.front': ep['image'][i].copy(),
                                       'action': ep['action'][i].copy(), 'task': TASK})
            dataset.save_episode()
            print(output.name, path.name)
    finally:
        dataset.finalize()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--raw', default='raw_episodes')
    p.add_argument('--out', default='data')
    p.add_argument('--validation-fraction', type=float, default=0.2)
    args = p.parse_args()
    if not 0 < args.validation_fraction < 1:
        p.error('validation-fraction must be in (0, 1)')
    files = sorted(Path(args.raw).glob('episode_*.npz'))
    if len(files) < 2:
        p.error('Need at least 2 successful episodes (pipeline smoke test only; collect more for training)')
    counts = {str(f): validate(f) for f in files}
    random.Random(42).shuffle(files)
    nval = max(1, min(len(files) - 1, round(len(files) * args.validation_fraction)))
    val, train = files[:nval], files[nval:]
    out = Path(args.out).resolve()
    if (out / 'train').exists() or (out / 'validation').exists():
        p.error('Dataset already exists. Use a new --out directory; no automatic overwrite.')
    out.mkdir(parents=True, exist_ok=True)
    (out / 'split.json').write_text(json.dumps({
        'seed': 42, 'train': [str(f) for f in train], 'validation': [str(f) for f in val],
        'frames': counts, 'fps': FPS, 'joint_names': JOINTS,
        'success_label': 'manual operator verification; not automatic contact/height checking',
    }, indent=2), encoding='utf-8')
    convert(train, out / 'train', 'local/crane_cube_train')
    convert(val, out / 'validation', 'local/crane_cube_validation')
    print('Converted. Entire episodes were split; no frame-level leakage.')


if __name__ == '__main__':
    main()
