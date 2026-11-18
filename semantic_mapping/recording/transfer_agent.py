"""JSON/binary file transfer protocol, invoked over SSH by pull_session.py."""
import fnmatch
import hashlib
import json
import os
import sys


def digest(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(4*1024*1024), b''):
            h.update(b)
    return h.hexdigest()


def main():
    root = os.path.realpath(sys.argv[1])

    def path(n):
        p = os.path.realpath(os.path.join(root, n))
        if os.path.isabs(n) or '..' in n.split('/') or os.path.commonpath([root,p]) != root or not os.path.isfile(p):
            raise ValueError('Invalid source file')
        return p

    def meta(n):
        s = os.stat(path(n))
        return dict(name=n, size=s.st_size, mtime_ns=s.st_mtime_ns)

    def send(d):
        sys.stdout.write(json.dumps(d)+'\n')
        sys.stdout.flush()

    for line in sys.stdin:
        try:
            q = json.loads(line)
            cmd = q['cmd']
            if cmd == 'list':
                names = sorted(os.listdir(root))
                selected, skipped = [], []
                folders = ['']+[n for n in names if os.path.isdir(os.path.join(root,n))
                               and not os.path.islink(os.path.join(root,n)) and not n.startswith('.')]
                for folder in folders:
                    entries = os.listdir(os.path.join(root,folder))
                    if any(n.endswith('.bag.active') for n in entries):
                        skipped.append(folder or '.')
                        continue
                    bags = [n for n in entries if n.endswith('.bag') and
                            not n.endswith('.mapping.bag') and fnmatch.fnmatch(n,q['pattern'])]
                    if not bags:
                        continue
                    chosen = list(bags)
                    if q['sidecars']:
                        chosen += [n for n in entries if n.endswith(('.yaml','.pgm','.json','.mapping.bag'))
                                   and n != 'transfer-manifest.json']
                    selected += [os.path.join(folder,n).replace(os.sep,'/') for n in sorted(set(chosen))]
                is_session = (os.path.basename(root).startswith(('session_','validation_')) or
                              any(n.endswith('.session.json') for n in names))
                group = os.path.basename(root) if is_session else 'legacy_'+os.path.basename(root)
                send(dict(files=[meta(n) for n in selected], root_group=group, skipped_active=skipped))
            elif cmd == 'hash':
                m = meta(q['name']); h = digest(path(q['name']))
                if m != meta(q['name']):
                    raise ValueError('Source changed while hashing')
                send(dict(m, sha256=h))
            elif cmd == 'read':
                n = q['name']; m = meta(n); off = q['offset']
                if m != q['meta'] or not 0 <= off <= m['size']:
                    raise ValueError('Source changed or invalid resume offset')
                with open(path(n), 'rb') as f:
                    f.seek(off); remaining = m['size']-off
                    send(dict(bytes=remaining))
                    while remaining:
                        b = f.read(min(1024*1024, remaining))
                        if not b:
                            raise RuntimeError('Unexpected source EOF')
                        sys.stdout.buffer.write(b); remaining -= len(b)
                    sys.stdout.buffer.flush()
                send(dict(stable=m == meta(n)))
            elif cmd == 'delete':
                for item in q['files']:
                    if meta(item['name']) != item['meta'] or digest(path(item['name'])) != item['sha256']:
                        raise ValueError('Source changed; no files deleted')
                removed = []
                for item in q['files']:
                    if meta(item['name']) != item['meta']:
                        raise ValueError('Source changed during deletion; removed: '+repr(removed))
                    os.unlink(path(item['name'])); removed.append(item['name'])
                send(dict(deleted=removed))
            elif cmd == 'close':
                send(dict(closed=True)); return
            else:
                raise ValueError('Unknown command')
        except Exception as e:
            send(dict(error=str(e)))


if __name__ == '__main__':
    main()
