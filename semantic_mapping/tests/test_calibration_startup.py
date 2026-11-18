import math
from pathlib import Path
import subprocess
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from calibration_schema import load_calibration, transform_error


class InstallationTests(unittest.TestCase):
    def test_quaternion_sign_and_live_offsets(self):
        expected=dict(xyz=[.1,0.,.1],quaternion_xyzw=[0.,0.,0.,1.])
        self.assertEqual(transform_error(expected['xyz'],[0.,0.,0.,-1.],expected),(0.,0.))
        distance,angle=transform_error([.12,0.,.1],[0.,0.,math.sin(.02),math.cos(.02)],expected)
        self.assertAlmostEqual(distance,.02)
        self.assertAlmostEqual(angle,.04)
        with self.assertRaises(ValueError):
            transform_error([float('nan'),0.,.1],[0.,0.,0.,1.],expected)

    def test_launch_uses_configured_quaternion_and_parent_child(self):
        path=ROOT/'recording/calibration.json'
        item=load_calibration(str(path))['transforms'][0]
        args=subprocess.check_output([sys.executable,str(ROOT/'recording/publish_calibration.py'),
                                      '--camera-args',str(path)]).decode().split()
        self.assertEqual(args[-2:],['base_link','camera_link'])
        self.assertEqual([float(v) for v in args[:-2]],item['xyz']+item['quaternion_xyzw'])

    def test_bundle_pins_launch_and_calibration(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location('startup_bundle',str(ROOT/'recording/recorder_bundle.py'))
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        self.assertIn('record_sources.launch',module.bundle())
        self.assertIn('calibration.json',module.bundle())
