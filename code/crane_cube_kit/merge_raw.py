"""Copy successful raw episodes into a NEW dataset source without deleting originals."""
import argparse
import hashlib
from pathlib import Path
import shutil

from convert_dataset import validate


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--inputs', nargs='+', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    if args.out.exists():
        p.error('Output exists; use a new folder')
    files = []
    for root in args.inputs:
        selected = sorted(root.glob('episode_*.npz'))
        if not selected:
            p.error('No episodes in ' + str(root))
        files.extend(selected)
    # Validate all inputs before creating output.
    for path in files:
        validate(path)
    args.out.mkdir(parents=True, exist_ok=False)
    hashes = set()
    count = 0
    for path in files:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest in hashes:
            continue
        hashes.add(digest)
        target = args.out / path.name
        if target.exists():
            target = args.out / (path.stem + '_' + digest[:12] + path.suffix)
        # Exclusive creation protects data if another process unexpectedly writes here.
        with path.open('rb') as source, target.open('xb') as output:
            shutil.copyfileobj(source, output)
        count += 1
    print(f'Copied {count} distinct successful episodes to {args.out}. Originals unchanged.')


if __name__ == '__main__':
    main()
