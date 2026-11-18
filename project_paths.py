"""Project-relative paths shared by launchers, viewers and transfer tools."""
from __future__ import unicode_literals
import os
import json

PROJECT_ROOT = os.path.dirname(os.path.realpath(__file__))


def configured_path(key, environment, default):
    config_file = os.environ.get('BAGTOOLS_PATHS_FILE', os.path.join(PROJECT_ROOT, 'paths.local.json'))
    config = {}
    if os.path.isfile(config_file):
        with open(config_file) as stream:
            config = json.load(stream)
    path = next((os.environ[name] for name in environment if name in os.environ),
                config.get(key, default))
    path = os.path.expanduser(path)
    if not os.path.isabs(path):
        path = os.path.join(PROJECT_ROOT, path)
    return os.path.abspath(path)


def default_data_dir():
    return configured_path('data_dir', ('BAGTOOLS_DATA_DIR', 'LOCAL_DIR'), 'bags')


def models_dir():
    return configured_path('models_dir', ('BAGTOOLS_MODELS_DIR',), 'models')


if __name__ == '__main__':
    print(default_data_dir())
