"""CPU ACT inference over loopback HTTP; run in the LeRobot environment."""
import argparse
import base64
from http.server import BaseHTTPRequestHandler, HTTPServer
import io
import json
import numpy as np
from PIL import Image
from common import IMAGE_SIZE, TASK


def configure_action_steps(policy, steps):
    """Change ACT queue length without changing trained weights or normalization."""
    if steps is None:
        return
    if not isinstance(steps, int) or not 1 <= steps <= policy.config.chunk_size:
        raise ValueError(f'action-steps must be 1..{policy.config.chunk_size}')
    if policy.config.temporal_ensemble_coeff is not None and steps != 1:
        raise ValueError('Temporal ensemble requires action-steps=1')
    policy.config.n_action_steps = steps
    policy.reset()


def configure_temporal_ensemble(policy, coefficient, factory):
    if coefficient is None:
        return
    if not np.isfinite(coefficient) or coefficient < 0:
        raise ValueError('temporal ensemble coefficient must be finite and >= 0')
    policy.config.n_action_steps = 1
    policy.config.temporal_ensemble_coeff = coefficient
    policy.temporal_ensembler = factory(coefficient, policy.config.chunk_size)
    policy.reset()


def load_policy(checkpoint, threads, action_steps=None, temporal_ensemble_coeff=None):
    import torch
    from lerobot.policies.act.modeling_act import ACTPolicy, ACTTemporalEnsembler
    from lerobot.policies.factory import make_pre_post_processors
    torch.set_num_threads(threads)
    policy = ACTPolicy.from_pretrained(checkpoint)
    policy.to('cpu').eval()
    policy.config.device = 'cpu'
    if temporal_ensemble_coeff is not None and action_steps not in (None, 1):
        raise ValueError('Temporal ensembling requires action-steps=1')
    configure_action_steps(policy, action_steps)
    configure_temporal_ensemble(policy, temporal_ensemble_coeff, ACTTemporalEnsembler)
    pre, post = make_pre_post_processors(policy_cfg=policy.config, pretrained_path=checkpoint,
        preprocessor_overrides={'device_processor': {'device': 'cpu'}})
    policy.reset()
    return policy, pre, post


def predict(policy, pre, post, state, rgb):
    import torch
    state = np.asarray(state, dtype=np.float32)
    if state.shape != (5,) or not np.isfinite(state).all():
        raise ValueError('Invalid state')
    if rgb.shape != (IMAGE_SIZE, IMAGE_SIZE, 3):
        raise ValueError('Invalid camera shape')
    # Processor adds the batch dimension and applies TRAINING normalization.
    obs = {'observation.state': torch.from_numpy(state.copy()),
           'observation.images.front': torch.from_numpy(rgb.copy()).permute(2, 0, 1).float() / 255.,
           'task': TASK}
    with torch.inference_mode():
        action = post(policy.select_action(pre(obs)))
    result = action.detach().cpu().numpy().reshape(-1)
    if result.shape != (5,) or not np.isfinite(result).all():
        raise ValueError('Invalid policy output')
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--checkpoint', required=True)
    p.add_argument('--port', type=int, default=8766)
    p.add_argument('--threads', type=int, default=4)
    p.add_argument('--action-steps', type=int, default=None,
                   help='Use 1..chunk_size predicted actions before observing again (default: checkpoint setting)')
    p.add_argument('--temporal-ensemble-coeff', type=float, default=None,
                   help='ACT temporal ensembling: overlap predictions every step; requires action-steps=1')
    args = p.parse_args()
    if args.threads < 1:
        p.error('threads must be positive')
    if args.temporal_ensemble_coeff is not None and args.action_steps not in (None, 1):
        p.error('--temporal-ensemble-coeff requires --action-steps 1 (or omit --action-steps)')
    policy, pre, post = load_policy(args.checkpoint, args.threads, args.action_steps, args.temporal_ensemble_coeff)
    print(f'ACT chunk_size={policy.config.chunk_size}, n_action_steps={policy.config.n_action_steps}', flush=True)
    print(f'ACT temporal_ensemble_coeff={policy.config.temporal_ensemble_coeff}', flush=True)
    step_index = [0]

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 2_000_000:
                    raise ValueError('Invalid request length')
                body = json.loads(self.rfile.read(length))
                if self.path == '/reset':
                    policy.reset()
                    step_index[0] = 0
                    response = {'ok': True}
                elif self.path == '/predict':
                    rgb = np.array(Image.open(io.BytesIO(base64.b64decode(body['image']))).convert('RGB'))
                    response = {
                        'action': predict(policy, pre, post, body['state'], rgb).tolist(),
                        'policy_step': step_index[0],
                        'chunk_size': policy.config.chunk_size,
                        'n_action_steps': policy.config.n_action_steps,
                        'temporal_ensemble_coeff': policy.config.temporal_ensemble_coeff,
                    }
                    step_index[0] += 1
                else:
                    raise ValueError('Unknown endpoint')
                data = json.dumps(response).encode()
                self.send_response(200)
            except Exception as e:
                data = json.dumps({'error': str(e)}).encode()
                self.send_response(400)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, fmt, *values):
            pass

    # Single-threaded: policy's action queue belongs to exactly one episode/client.
    server = HTTPServer(('127.0.0.1', args.port), Handler)
    server.timeout = 20
    print(f'CPU policy ready: http://127.0.0.1:{args.port} (same container only)', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
