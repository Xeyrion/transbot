import os
from pathlib import Path
import sys
import unittest
import importlib.util
import tempfile
import json
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from workbench import launch_spec
from project_paths import PROJECT_ROOT


class WorkbenchTests(unittest.TestCase):
    def test_default_paths_follow_project_location_and_shared_overrides(self):
        with tempfile.TemporaryDirectory(prefix='semantic mapping ') as folder:
            source=Path(PROJECT_ROOT)/'project_paths.py'
            copied=Path(folder)/source.name;copied.write_bytes(source.read_bytes())
            spec=importlib.util.spec_from_file_location('relocated_paths', str(copied))
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            with patch.dict(os.environ, {}, clear=True):
                self.assertEqual(module.default_data_dir(), str(Path(folder).resolve()/'bags'))
            home={'HOME':folder, 'USERPROFILE':folder}
            with patch.dict(os.environ, dict(home, LOCAL_DIR='~/custom_bags'), clear=True):
                self.assertEqual(module.default_data_dir(), str(Path(folder)/'custom_bags'))
            with patch.dict(os.environ, dict(home, LOCAL_DIR='/unused', BAGTOOLS_DATA_DIR='~/chosen'), clear=True):
                self.assertEqual(module.default_data_dir(), str(Path(folder)/'chosen'))
            (Path(folder)/'paths.local.json').write_text(json.dumps(
                dict(data_dir=str(Path(folder)/'external'/'bags'), models_dir='weights')))
            with patch.dict(os.environ, {}, clear=True):
                self.assertEqual(module.default_data_dir(), str(Path(folder)/'external'/'bags'))
                self.assertEqual(module.models_dir(), str(Path(folder)/'weights'))
            with patch.dict(os.environ, {'BAGTOOLS_DATA_DIR':str(Path(folder)/'override')}, clear=True):
                self.assertEqual(module.default_data_dir(), str(Path(folder)/'override'))

    def test_both_tools_share_one_default_data_directory(self):
        directory=os.path.abspath('/tmp/workbench_data')
        recording,rec_env=launch_spec('record',directory)
        review,review_env=launch_spec('review',directory)
        self.assertEqual(review[-1],directory)
        self.assertEqual(rec_env['LOCAL_DIR'],directory)
        self.assertEqual(review_env['LOCAL_DIR'],directory)
        self.assertTrue(recording[1].endswith('record_gui.py'))
        self.assertTrue(review[1].endswith('bag_quality_gui.py'))


if __name__=='__main__':unittest.main()
