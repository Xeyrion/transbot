#!/usr/bin/env bash
# Install the single workbench entry for the current Linux desktop user.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || printf '%s/Desktop' "$HOME")"
python3 - "$HERE" "$DESKTOP_DIR" <<'PY'
from pathlib import Path
import sys
root,desktop=map(Path,sys.argv[1:])
def quote(s):
    return '"'+s.replace('\\','\\\\').replace('"','\\"').replace('`','\\`').replace('$','\\$').replace('%','%%')+'"'
for suffix,title,comment,entry in [
        ('', 'Transbot 数据工作台', '机器人录制与数据阅读统一入口', 'run.sh'),
        ('-en', 'Transbot Data Workbench (EN)', 'Robot recording and session review', 'run_en.sh')]:
    script=root/entry
    if not script.exists():
        sys.exit('missing launcher: %s' % script)
    content='\n'.join(['[Desktop Entry]','Type=Application','Name='+title,
        'Comment='+comment,
        'Exec=/bin/bash '+quote(str(script)),'Path='+str(root),
        'Terminal=false','Icon=applications-science','Categories=Science;',
        'StartupNotify=true',''])
    for folder in (desktop,Path.home()/'.local/share/applications'):
        folder.mkdir(parents=True,exist_ok=True)
        p=folder/('transbot-data-workbench'+suffix+'.desktop')
        p.write_text(content,encoding='utf-8');p.chmod(0o755)
        print(p)

PY
for entry in "$DESKTOP_DIR"/transbot-data-workbench{,-en}.desktop; do
if command -v gio >/dev/null 2>&1; then
  if [ -S "/run/user/$(id -u)/bus" ]; then
    DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus" \
      gio set "$entry" metadata::trusted true || true
  fi
fi
if command -v desktop-file-validate >/dev/null 2>&1; then
  desktop-file-validate "$entry"
fi
done
echo '日常入口：控制机桌面的“Transbot 数据工作台”。'
