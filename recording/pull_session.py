#!/usr/bin/env python3
"""Resume .part files over SSH, verify SHA-256, optionally delete verified batch."""
import argparse
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def manifest_sha(local_root, group, name):
    """sha256 an earlier verified transfer recorded for `name`, if any.

    The manifest is written after each file is hashed, so it is a cheap way to
    tell "we already pulled this one" apart from "a same-sized stranger sits
    in the folder". It is a hint only -- every real operation re-hashes.
    """
    path = os.path.join(local_root, group, 'transfer-manifest.json')
    try:
        with open(path) as f:
            for item in json.load(f).get('files', []):
                if item.get('name') == name and item.get('status') == 'verified':
                    return item.get('sha256')
    except (IOError, OSError, ValueError):
        pass
    return None


def local_state(path, meta, local_root, group, name):
    """Cheap local verdict: no remote read, no full-file hash.

    missing / partial / size-mismatch are conclusive. present vs verified only
    says whether the transfer manifest vouches for the copy. `partial` means a
    .part is mid-flight, which the picker has to show as progress rather than
    as "not here yet".
    """
    if not os.path.exists(path):
        part = path + '.part'
        if os.path.exists(part):
            return 'partial', False, os.path.getsize(part)
        return 'missing', False, 0
    if os.path.getsize(path) != meta['size']:
        return 'size-mismatch', False, 0
    return 'present', bool(manifest_sha(local_root, group, name)), 0


def destination_lock(directory):
    os.makedirs(directory, exist_ok=True)
    handle = open(os.path.join(directory, '.transfer.lock'), 'a+b')
    handle.seek(0)
    if os.name == 'nt':
        import msvcrt
        if not os.path.getsize(handle.name):
            handle.write(b'0'); handle.flush(); handle.seek(0)
        msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
    else:
        import fcntl
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    return handle


def digest(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(4*1024*1024), b''):
            h.update(b)
    return h.hexdigest()


class Agent:
    def __init__(self, args):
        with open(os.path.join(HERE, 'transfer_agent.py')) as f:
            code = f.read()
        cmd = ['ssh', '-T', '-p', str(args.port), '-o', 'ConnectTimeout=10',
               '-o', 'ServerAliveInterval=15', '-o', 'ServerAliveCountMax=4']
        if args.jump:
            cmd += ['-J', args.jump]
        cmd += [args.robot, 'python3 -u -c %s %s' % (shlex.quote(code), shlex.quote(args.remote))]
        env = dict(os.environ)
        if shutil.which('sshpass') and env.get('ROBOT_PASS'):
            env['SSHPASS'] = env['ROBOT_PASS']; cmd = ['sshpass', '-e'] + cmd
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env)

    def request(self, **q):
        self.proc.stdin.write((json.dumps(q)+'\n').encode()); self.proc.stdin.flush()
        return self.reply()

    def reply(self):
        line = self.proc.stdout.readline()
        if not line:
            raise RuntimeError('SSH disconnected; rerun to resume .part files')
        d = json.loads(line.decode())
        if 'error' in d:
            raise RuntimeError(d['error'])
        return d


def emit_listing(files, args, listing, relative, target):
    """JSON for the GUI picker: name, size, and whether we already have it."""
    rows = []
    for m in files:
        n = m['name']
        dst = target(n)
        state, recorded, partial = local_state(dst, m, args.local,
                                               relative(n).split('/')[0], n)
        rows.append(dict(name=n, size=m['size'], mtime_ns=m['mtime_ns'],
                         local=state, recorded=recorded, partial_size=partial,
                         local_path=dst))
    try:
        free = shutil.disk_usage(args.local).free if os.path.isdir(args.local) else None
    except OSError:
        free = None
    print(json.dumps(dict(files=rows, remote=args.remote, local_root=args.local,
                          root_group=listing['root_group'], free=free,
                          skipped_active=listing.get('skipped_active', []))))


def delete_verified(a, files, target):
    """Delete remote copies only where a hash-identical local copy exists.

    Deliberately narrower than `-d`: `-d` removes whatever the batch listed,
    including files skipped because they were already local. This refuses to
    remove anything it cannot prove is safe, and reports the rest instead of
    aborting the whole run.
    """
    ready, blocked = [], []
    for m in files:
        n = m['name']
        remote = a.request(cmd='hash', name=n)
        if any(remote[k] != m[k] for k in m):
            raise RuntimeError('Source changed: ' + n)
        dst = target(n)
        if not os.path.exists(dst):
            why = 'no local copy'
        elif os.path.getsize(dst) != m['size']:
            why = 'local size differs'
        elif digest(dst) != remote['sha256']:
            why = 'local content differs'
        else:
            ready.append(dict(name=n, meta=m, sha256=remote['sha256']))
            continue
        blocked.append(n)
        print('Kept on robot: %s -- %s' % (n, why), flush=True)
    if not ready:
        raise RuntimeError('Nothing safe to delete: no selected bag has a verified local copy')
    deleted = a.request(cmd='delete', files=ready)['deleted']
    print('Deleted %d of %d remote files; kept %d without a verified local copy.'
          % (len(deleted), len(files), len(blocked)), flush=True)
    return deleted


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--robot', default=os.environ.get('ROBOT', 'jetson@192.168.1.11'))
    ap.add_argument('--port', type=int, default=22)
    ap.add_argument('--jump')
    ap.add_argument('-r', '--remote', default=os.environ.get('REMOTE_DIR', '/home/jetson/bags'))
    ap.add_argument('-l', '--local', default=os.environ.get('LOCAL_DIR', os.path.expanduser('~/robot_research/bagtools/bags')))
    ap.add_argument('-p', '--pattern', default='cam_*.bag')
    ap.add_argument('--no-sidecars', action='store_true')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--list-json', action='store_true',
                    help='machine-readable listing with local state, for the GUI')
    ap.add_argument('--names', nargs='+', metavar='NAME',
                    help='handle exactly these entries (as printed by --list-json)')
    ap.add_argument('--delete-only', action='store_true',
                    help='delete selected remote bags, but only where a verified '
                         'local copy exists; nothing is transferred')
    ap.add_argument('-d', '--delete', action='store_true')
    args = ap.parse_args()
    lock = None if (args.list or args.list_json) else destination_lock(args.local)
    a = Agent(args)
    try:
        listing = a.request(cmd='list', pattern=args.pattern, sidecars=not args.no_sidecars)
        files = listing['files']
        if args.names:
            wanted = set(args.names)
            files = [f for f in files if f['name'] in wanted]
            for name in sorted(wanted - set(f['name'] for f in files)):
                print('Not in listing (moved or already gone): ' + name,
                      file=sys.stderr, flush=True)
        if not args.list_json:
            for name in listing.get('skipped_active', []):
                print('Skipping active session: '+name, flush=True)
        def relative(name):
            return name if '/' in name else listing['root_group']+'/'+name
        def target(name):
            p = os.path.abspath(os.path.join(args.local, *relative(name).split('/')))
            if os.path.commonpath([os.path.abspath(args.local), p]) != os.path.abspath(args.local):
                raise ValueError('Invalid destination path')
            return p
        if args.list_json:
            emit_listing(files, args, listing, relative, target)
            return
        if not any(f['name'].endswith('.bag') for f in files):
            raise RuntimeError('No matching finalized bags')
        print('%d files, %.2f GiB' % (len(files), sum(f['size'] for f in files)/2**30), flush=True)
        if args.list:
            for f in files: print('%12d %s' % (f['size'], f['name']))
            return
        if args.delete_only:
            delete_verified(a, files, target)
            return
        os.makedirs(args.local, exist_ok=True)
        manifest = dict(source=args.robot, remote=args.remote, files=[], deleted=[])

        def save():
            groups = {}
            for item in manifest['files']:
                group = relative(item['name']).split('/')[0]
                groups.setdefault(group, []).append(item)
            for group, items in groups.items():
                manifest_path = os.path.join(args.local, group, 'transfer-manifest.json')
                data = dict(source=args.robot, remote=args.remote, files=items,
                            deleted=[n for n in manifest['deleted'] if relative(n).split('/')[0] == group])
                with open(manifest_path+'.tmp', 'w') as f:
                    json.dump(data, f, ensure_ascii=False, indent=2)
                    f.flush(); os.fsync(f.fileno())
                os.replace(manifest_path+'.tmp', manifest_path)

        for m in files:
            n = m['name']; dst = target(n); part = dst+'.part'
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            remote = a.request(cmd='hash', name=n)
            if any(remote[k] != m[k] for k in m):
                raise RuntimeError('Source changed: '+n)
            if os.path.exists(dst):
                if os.path.getsize(dst) != m['size'] or digest(dst) != remote['sha256']:
                    raise RuntimeError('Existing destination differs; preserved: '+dst)
                print('Verified, skipping '+n, flush=True)
            else:
                partial_meta = part+'.json'
                if os.path.exists(part) and os.path.exists(partial_meta):
                    with open(partial_meta) as f:
                        previous = json.load(f)
                    if previous != remote:
                        os.replace(part, part+'.stale-'+str(int(time.time())))
                with open(partial_meta, 'w') as f:
                    json.dump(remote, f)
                off = os.path.getsize(part) if os.path.exists(part) else 0
                if off > m['size']: raise RuntimeError('Oversized partial: '+part)
                if shutil.disk_usage(args.local).free < m['size']-off+1024**3:
                    raise RuntimeError('Need more destination space (1 GiB reserve)')
                count = a.request(cmd='read', name=n, offset=off, meta=m)['bytes']
                started, last, done = time.monotonic(), 0, 0
                with open(part, 'ab') as f:
                    while done < count:
                        b = a.proc.stdout.read(min(1024*1024, count-done))
                        if not b: raise RuntimeError('Disconnected; partial retained for resume')
                        f.write(b); done += len(b); now = time.monotonic()
                        if now-last >= 5:
                            print('%s %.1f%% %.1f MiB/s' % (n, 100*(off+done)/max(1,m['size']),
                                  done/2**20/max(.01, now-started)), flush=True); last = now
                    f.flush(); os.fsync(f.fileno())
                if not a.reply()['stable'] or digest(part) != remote['sha256']:
                    os.replace(part, part+'.failed-'+str(int(time.time())))
                    raise RuntimeError('Verification failed; failed copy preserved; rerun to restart: '+n)
                os.replace(part, dst)
                os.unlink(partial_meta)
            os.utime(dst, ns=(m['mtime_ns'], m['mtime_ns']))
            manifest['files'].append(dict(name=n, meta=m, sha256=remote['sha256'], status='verified'))
            save()
        if args.delete:
            for f in manifest['files']:
                if digest(target(f['name'])) != f['sha256']:
                    raise RuntimeError('Local backup changed; source retained')
            manifest['deleted'] = a.request(cmd='delete', files=manifest['files'])['deleted']; save()
        print('Verified %d files; deleted %d source files.' % (len(files), len(manifest['deleted'])), flush=True)
    finally:
        if a.proc.poll() is None:
            a.proc.terminate()
        a.proc.wait()
        if lock is not None:
            lock.close()


if __name__ == '__main__':
    main()
