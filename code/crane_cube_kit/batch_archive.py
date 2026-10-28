"""Persist every recorded trial; rejected/partial data stays outside training roots."""
from datetime import datetime
import json
from pathlib import Path

import numpy as np
from common import FPS, JOINTS, TASK


def save_trial(out, rejected_directory, samples, cubes, cube_stamps, stages, initial_cube, settings, report):
    if not samples:
        return None
    passed = bool(report['passed'])
    directory = Path(out) if passed else Path(rejected_directory)
    directory.mkdir(parents=True, exist_ok=True)
    prefix = 'episode_' if passed else 'rejected_episode_'
    path = directory / (prefix + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.npz')
    times = np.array([s[0] for s in samples])
    with path.open('xb') as stream:
        np.savez_compressed(stream,
            state=np.stack([s[1] for s in samples]).astype(np.float32),
            action=np.stack([s[3] for s in samples]).astype(np.float32),
            image=np.stack([s[2] for s in samples]), time=times - times[0],
            joint_stamp=np.array([s[4] for s in samples]),
            image_stamp=np.array([s[5] for s in samples]),
            joint_names=np.array(JOINTS), fps=np.array(FPS), success=np.array(passed), task=np.array(TASK),
            source=np.array('moveit_varied_reference_lookahead_0.2s_auto_geometry_v2'),
            cube_position_world=np.asarray(cubes), cube_stamp=np.asarray(cube_stamps),
            initial_cube_world=np.asarray(initial_cube), stage=np.asarray(stages),
            collection_settings=np.array(json.dumps(settings)),
            automatic_verification=np.array(json.dumps(report)))
    return path
