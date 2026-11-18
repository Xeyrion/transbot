#!/usr/bin/env python3
"""Batch entry point for the same current-map workflow used by the workbench."""
import argparse
from pathlib import Path
from mapping_job import recording_groups,map_artifacts,new_job,MappingJob
from viewer_core import session_files,map_manifest_path


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('root',nargs='?',default=str(Path(__file__).resolve().parent/'bags'))
    ap.add_argument('--replace',action='store_true',help='Replace current maps, archiving previous versions')
    ap.add_argument('--dry-run',action='store_true');args=ap.parse_args()
    root=Path(args.root).expanduser().resolve()
    if root.is_file():groups=[session_files(str(root))]
    else:
        folders=[root]+[p for p in root.iterdir() if p.is_dir() and not p.is_symlink() and p.name not in ('maps','archive','results') and not p.name.startswith('.')]
        groups=[g for folder in folders for g in recording_groups(str(folder))]
    failures=0
    for files in groups:
        first=files[0]
        if list(Path(first).parent.glob('*.bag.active')):
            print('SKIP active recording:',first);continue
        if map_artifacts(first) and not args.replace:
            print('SKIP current map:',map_manifest_path(first));continue
        print('BUILD:',first,flush=True)
        if args.dry_run:continue
        job=new_job(first)
        try:
            MappingJob(first,str(job),args.replace).run()
            print('MAP:',map_manifest_path(first),flush=True)
        except (Exception,KeyboardInterrupt) as exc:
            failures+=1;print('FAILED:',first,str(exc),'LOG:',job,flush=True)
    return 1 if failures else 0


if __name__=='__main__':raise SystemExit(main())
