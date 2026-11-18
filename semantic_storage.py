"""Store the base map used for semantic projection alongside its run."""
import hashlib
import os
import shutil


def digest(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1048576), b''):
            h.update(chunk)
    return h.hexdigest()


def snapshot_map(folder, linked):
    import yaml
    source_yaml = linked['yaml']
    with open(source_yaml) as f:
        info = yaml.safe_load(f)
    image = info['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(source_yaml), image)
    elif not os.path.isfile(image):
        image = os.path.join(os.path.dirname(source_yaml), os.path.basename(image))

    snapshot = os.path.join(folder, 'base_map')
    os.mkdir(snapshot)
    info['image'] = 'map' + os.path.splitext(image)[1]
    image_path = os.path.join('base_map', info['image'])
    yaml_path = os.path.join('base_map', 'map.yaml')
    shutil.copy2(image, os.path.join(folder, image_path))
    with open(os.path.join(folder, yaml_path), 'w') as f:
        yaml.safe_dump(info, f)
    paths = [yaml_path, image_path]
    trajectory = None
    if linked.get('trajectory'):
        trajectory = os.path.join('base_map', 'trajectory.bag')
        shutil.copy2(linked['trajectory'], os.path.join(folder, trajectory))
        paths.append(trajectory)
    return dict(yaml=yaml_path, trajectory=trajectory,
                sha256={path: digest(os.path.join(folder, path)) for path in paths})
