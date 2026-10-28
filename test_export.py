import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

from export_from_container import build_sources, write_archive


class ExportTests(unittest.TestCase):
    def test_requires_real_checkpoint_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); root.joinpath('workspace').mkdir(); root.joinpath('bundle').mkdir()
            with self.assertRaises(FileNotFoundError):
                build_sources(root/'bundle', root/'workspace', root/'missing')

    def test_source_overlay_model_processors_data_option_and_hashes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); bundle=root/'bundle'; work=root/'work'; model=root/'model'
            bundle.mkdir(); work.mkdir(); model.mkdir()
            (bundle/'code/crane_cube_kit').mkdir(parents=True)
            (bundle/'code/crane_cube_kit/robot.py').write_text('snapshot')
            (bundle/'README_JA.md').write_text('README')
            (work/'robot.py').write_text('actual source')
            for name in ['config.json','model.safetensors','policy_preprocessor.json']:
                (model/name).write_bytes(b'test fixture only')
            (work/'raw_auto_varied/batch_logs/trial_00000').mkdir(parents=True)
            (work/'raw_auto_varied/episode_one.npz').write_bytes(b'raw fixture')
            (work/'raw_auto_varied/batch_logs/trial_00000/rejected_episode_one.npz').write_bytes(b'rejected')
            sources=build_sources(bundle,work,model,False)
            self.assertEqual(sources['code/crane_cube_kit/robot.py'], work/'robot.py')
            self.assertTrue(any(k.endswith('policy_preprocessor.json') for k in sources))
            self.assertFalse(any(k.endswith('.npz') for k in sources))
            sources=build_sources(bundle,work,model,True)
            self.assertTrue(any(k.endswith('episode_one.npz') for k in sources))
            self.assertFalse(any('rejected_episode' in k for k in sources))
            output=root/'out.zip'; manifest=write_archive(output,sources,dict(model_included=True))
            with zipfile.ZipFile(output) as z:
                self.assertIsNone(z.testzip())
                for entry in manifest['files']:
                    data=z.read('crane_cube_submission/'+entry['path'])
                    self.assertEqual(hashlib.sha256(data).hexdigest(),entry['sha256'])
            with self.assertRaises(FileExistsError): write_archive(output,sources,{})


if __name__=='__main__': unittest.main()
