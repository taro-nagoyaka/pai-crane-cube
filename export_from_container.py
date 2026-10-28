"""Create a complete submission ZIP by adding real model files from the user's container.

This script is packaging only: it never moves/deletes data, runs training, or controls ROS.
Run with /usr/bin/python3; standard library only. Its directory is the unpacked submission.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import zipfile


def build_sources(bundle, workspace, checkpoint, include_data=False):
    required = [checkpoint / 'config.json', checkpoint / 'model.safetensors']
    for path in required:
        if not path.is_file():
            raise FileNotFoundError('必要な学習済みファイルがありません: ' + str(path))
    sources = {}
    for path in bundle.rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts and path.suffix != '.zip':
            sources[path.relative_to(bundle).as_posix()] = path
    prefix = Path('code/crane_cube_kit')
    # Actual user-workspace source takes precedence over the supplied snapshot.
    # Submission helpers are retained when they do not exist in the workspace.
    for path in workspace.iterdir():
        if path.is_file() and path.suffix in ('.py', '.sh', '.cpp', '.md', '.txt'):
            sources[(prefix/path.name).as_posix()] = path
    def include_tree(root, relative):
        if root.is_dir():
            for path in root.rglob('*'):
                if path.is_file() and '__pycache__' not in path.parts:
                    sources[(relative/path.relative_to(root)).as_posix()] = path
    # Keep checkpoint relative paths compatible with policy_server.py examples.
    include_tree(checkpoint, prefix/'outputs/act_varied_novae_18000/checkpoints/last/pretrained_model')
    include_tree(workspace/'teacher_ws/src', prefix/'teacher_ws/src')
    for name in ('cube_camera.launch.py', 'table_camera.sdf', 'lift_teacher.launch.py'):
        path = workspace/'generated'/name
        if path.is_file(): sources[(prefix/'generated'/name).as_posix()] = path
    for name in ('diagnostics_varied_novae6000', 'diagnostics_varied_novae18000'):
        include_tree(workspace/name, prefix/name)
    for path in workspace.glob('deploy_*.csv'):
        sources[(prefix/path.name).as_posix()] = path
    split = workspace/'data_varied_v1/split.json'
    if split.is_file(): sources[(prefix/'data_varied_v1/split.json').as_posix()] = split
    if include_data:
        include_tree(workspace/'data_varied_v1', prefix/'data_varied_v1')
        for name in ('raw_auto_branch', 'raw_auto_varied'):
            for path in (workspace/name).glob('episode_*.npz'):
                sources[(prefix/name/path.name).as_posix()] = path
    return sources


def write_archive(output, sources, metadata):
    manifest = dict(metadata, files=[])
    with output.open('xb') as stream:
        with zipfile.ZipFile(stream, 'w', zipfile.ZIP_DEFLATED) as archive:
            for relative, path in sorted(sources.items()):
                digest = hashlib.sha256()
                with path.open('rb') as source, archive.open('crane_cube_submission/'+relative, 'w', force_zip64=True) as dest:
                    for block in iter(lambda: source.read(1024*1024), b''):
                        digest.update(block); dest.write(block)
                manifest['files'].append(dict(path=relative, bytes=path.stat().st_size, sha256=digest.hexdigest()))
            archive.writestr('crane_cube_submission/submission_manifest.json',
                             json.dumps(manifest, indent=2, ensure_ascii=False))
    return manifest


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace', type=Path, default=Path('/home/ubuntu/crane_cube_kit'))
    p.add_argument('--checkpoint', type=Path, default=None)
    p.add_argument('--out', type=Path, default=None)
    p.add_argument('--include-data', action='store_true', help='Also include training datasets and successful raw episodes')
    args = p.parse_args()
    bundle = Path(__file__).resolve().parent
    workspace = args.workspace.resolve()
    checkpoint = (args.checkpoint or workspace/'outputs/act_varied_novae_18000/checkpoints/last/pretrained_model').resolve()
    output = (args.out or workspace.parent/('crane_cube_submission_full_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.zip')).resolve()
    if not workspace.is_dir(): p.error('Workspace does not exist: ' + str(workspace))
    if output.exists(): p.error('Output exists; use a new --out name')
    sources = build_sources(bundle, workspace, checkpoint, args.include_data)
    # Never read the output archive into itself, even with a custom path.
    if output in [path.resolve() for path in sources.values()]: p.error('Output conflicts with an input file')
    write_archive(output, sources, dict(created=datetime.now().isoformat(),
        model_included=True, training_data_included=args.include_data,
        task_status='Learned policy reaches grasp, then stalls; full learned lift not achieved',
        checkpoint_source=str(checkpoint)))
    print('提出用ZIPを作成しました: ' + str(output))
    print('学習済み重みと前処理・後処理ファイルを同梱しました。元データは変更していません。')
    print('学習データ同梱: ' + str(args.include_data))


if __name__ == '__main__':
    main()
