import os
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from workbench import launch_spec


class WorkbenchTests(unittest.TestCase):
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
