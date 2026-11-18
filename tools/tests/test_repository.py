"""Verify that publication checks reject drift and files absent from Git."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    'repository_check', str(Path(__file__).resolve().parents[1] / 'check_repository.py'))
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


class RepositoryChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='transbot-doc-check-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        subprocess.check_call(['git', 'init', '-q', str(self.root)])
        self.project = dict(path='semantic_mapping', name_zh='研究', name_en='Research',
                            readme='semantic_mapping/README.md', scope_zh='演示', scope_en='Demo',
                            entrypoints=['semantic_mapping/run.sh'])
        self.write('docs/projects.json', json.dumps(dict(projects=[self.project])))
        self.write('semantic_mapping/README.md', '# Research\n')
        self.write('semantic_mapping/run.sh', '#!/bin/bash\n')
        for name, english in [('README.md', False), ('README.en.md', True)]:
            self.write(name, '# Projects\n' + checker.START + '\n' +
                       checker.project_table([self.project], english) + '\n' + checker.END + '\n')
        self.stage()

    def write(self, name, content):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8')

    def stage(self):
        subprocess.check_call(['git', '-C', str(self.root), 'add', '.'])

    def check(self, *args):
        output, errors = io.StringIO(), io.StringIO()
        with patch.object(checker, 'ROOT', self.root), patch.object(sys, 'argv', ['check'] + list(args)), \
                contextlib.redirect_stdout(output), contextlib.redirect_stderr(errors):
            code = checker.main()
        return code, errors.getvalue()

    def test_index_only_links_even_when_target_exists_locally(self):
        self.assertEqual(self.check()[0], 0)
        self.write('README.md', (self.root / 'README.md').read_text(encoding='utf-8') + '\n[Guide](local-only.md)\n')
        self.stage()
        self.write('local-only.md', '# Not published\n')
        code, error = self.check()
        self.assertEqual(code, 1)
        self.assertIn('missing from Git', error)

    def test_registry_drift_is_repaired_by_explicit_generation(self):
        self.project['scope_en'] = 'Updated scope'
        self.write('docs/projects.json', json.dumps(dict(projects=[self.project])))
        self.stage()
        self.assertIn('project table drift', self.check()[1])
        self.assertEqual(self.check('--write-index')[0], 0)
        self.assertIn('Stage the reviewed', self.check()[1])
        self.stage()
        self.assertEqual(self.check()[0], 0)

    def test_runtime_payload_and_missing_anchor_are_rejected(self):
        self.write('semantic_mapping/models/private.pt', 'not a model fixture')
        self.write('semantic_mapping/README.md', '# Research\n[Missing](#does-not-exist)\n')
        self.stage()
        code, errors = self.check()
        self.assertEqual(code, 1)
        self.assertIn('Runtime data', errors)
        self.assertIn('missing heading', errors)


if __name__ == '__main__':
    unittest.main()
