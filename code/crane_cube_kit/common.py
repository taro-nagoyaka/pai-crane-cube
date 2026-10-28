"""Shared schema: radians, absolute target joint positions, 10 Hz simulation time."""
import numpy as np

JOINTS = [f'crane_plus_joint{i}' for i in range(1, 5)] + ['crane_plus_joint_hand']
LOWER = np.array([-2.6179916667, -1.8247401917, -2.5565561289, -1.8400990761, -0.7089521433])
UPPER = np.array([2.6179916667, 1.7837249889, 2.4542799211, 1.7683661044, 0.6679369406])
FPS = 10
IMAGE_SIZE = 224
TASK = 'Grasp the wood cube and lift it near the upright arm height, then hold it.'


def reorder(names, values):
    by_name = dict(zip(names, values))
    return np.array([by_name[n] for n in JOINTS], dtype=np.float32)


def bounded_target(values):
    a = np.asarray(values, dtype=np.float64)
    if a.shape != (5,) or not np.isfinite(a).all():
        raise ValueError('Action must contain five finite joint positions')
    return np.clip(a, LOWER, UPPER)


def image_rgb(msg):
    """Respect ROS Image row padding; avoid cv_bridge / NumPy ABI conflicts."""
    from PIL import Image
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(msg.encoding.lower())
    if channels is None:
        raise ValueError(f'Unsupported camera encoding: {msg.encoding}')
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    arr = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    arr = arr[:, :, :3]
    if msg.encoding.lower().startswith('bgr'):
        arr = arr[:, :, ::-1]
    return np.asarray(Image.fromarray(arr).resize((IMAGE_SIZE, IMAGE_SIZE)), dtype=np.uint8).copy()
