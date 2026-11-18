#!/usr/bin/env python3
"""Check the publishable Git index, project entries and local document links.

Run after git add. --write-index updates the marked README project tables from
docs/projects.json. No ROS installation or robot connection is needed.
"""
import argparse
import ast
import json
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
START = '<!-- projects:start -->'
END = '<!-- projects:end -->'
DATA_EXTENSIONS = ('.bag', '.bag.active', '.pt', '.pth', '.onnx', '.weights',
                   '.npy', '.npz', '.part', '.log', '.zip', '.tgz', '.tar', '.gz')
MAX_FILE_BYTES = 12 * 1024 * 1024


def git_files():
    output = subprocess.check_output(['git', 'ls-files', '-z'], cwd=str(ROOT))
    return set(output.decode('utf-8').rstrip('\0').split('\0')) - {''}


def project_table(projects, english):
    key = 'en' if english else 'zh'
    lines = (['| Project | Directory | Scope |'] if english else
             ['| 项目 | 目录 | 当前范围 |']) + ['|---|---|---|']
    for project in projects:
        doc = project.get('readme_en', project['readme']) if english else project['readme']
        lines.append('| {} | [{}/]({}) | {} |'.format(
            project['name_' + key], project['path'], doc, project['scope_' + key]))
    return '\n'.join(lines)


def headings(text):
    """GitHub-style heading IDs for the plain Markdown used in this repo."""
    result, counts = set(), {}
    for title in re.findall(r'^#{1,6}\s+(.+?)\s*#*$', text, re.M):
        title = re.sub(r'<[^>]*>', '', title).lower()
        slug = re.sub(r'[^\w\s-]', '', title, flags=re.UNICODE).replace(' ', '-')
        count = counts.get(slug, 0)
        counts[slug] = count + 1
        result.add(slug + ('-' + str(count) if count else ''))
    return result


def check_links(name, files, errors):
    path = ROOT / name
    text = path.read_text(encoding='utf-8-sig')
    text = re.sub(r'^(```|~~~).*?^\1[^\n]*$', '', text, flags=re.M | re.S)
    for match in re.finditer(r'!?\[[^\]]*\]\(([^\n)]+)\)', text):
        href = match.group(1).strip().split(' "', 1)[0].strip('<>')
        parsed = urlsplit(href)
        if parsed.scheme or parsed.netloc:
            continue
        target = (path.parent / unquote(parsed.path)).resolve() if parsed.path else path
        try:
            relative = target.relative_to(ROOT).as_posix()
        except ValueError:
            errors.append('{}: link escapes the repository: {}'.format(name, href))
            continue
        published = relative in files or any(p.startswith(relative + '/') for p in files)
        if not target.exists() or not published:
            errors.append('{}: link is missing from Git: {}'.format(name, href))
        elif parsed.fragment and target.suffix == '.md':
            if unquote(parsed.fragment) not in headings(target.read_text(encoding='utf-8-sig')):
                errors.append('{}: missing heading: {}'.format(name, href))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write-index', action='store_true')
    args = parser.parse_args()
    projects = json.loads((ROOT / 'docs/projects.json').read_text(encoding='utf-8'))['projects']
    files, errors = git_files(), []
    if not args.write_index:
        unstaged = subprocess.check_output(
            ['git', 'diff', '--name-only', '-z'], cwd=str(ROOT)).decode('utf-8').strip('\0')
        if unstaged:
            errors.append('Stage the reviewed working-tree changes before checking the publishable index: ' +
                          ', '.join(unstaged.split('\0')))
    for name, english in [('README.md', False), ('README.en.md', True)]:
        path = ROOT / name
        text = path.read_text(encoding='utf-8')
        expected = START + '\n' + project_table(projects, english) + '\n' + END
        pattern = re.escape(START) + '.*?' + re.escape(END)
        if not re.search(pattern, text, re.S):
            errors.append(name + ': project table markers are missing')
        elif args.write_index:
            path.write_text(re.sub(pattern, lambda unused: expected, text, flags=re.S), encoding='utf-8')
        elif re.search(pattern, text, re.S).group() != expected:
            errors.append(name + ': project table drift; run --write-index and stage the README')
    for project in projects:
        required = [project['readme']] + project['entrypoints']
        if project.get('readme_en'):
            required.append(project['readme_en'])
        for name in required:
            if name not in files or not (ROOT / name).is_file():
                errors.append('Project entry/document is missing from Git: ' + name)
        if (ROOT / project['path'] / '.git').exists():
            errors.append('Nested source repository: ' + project['path'])
    for name in sorted(files):
        path = ROOT / name
        parts = path.parts[len(ROOT.parts):]
        if any(p in ('_local', '.slidep', '.workbuddy-ai') for p in parts):
            errors.append('Local-only directory in Git: ' + name)
        if (name.endswith(DATA_EXTENSIONS) or path.name == 'paths.local.json' or
                (name.startswith('semantic_mapping/') and len(parts) > 1 and
                 parts[1] in ('bags', 'models', 'datasets', 'results', 'archive'))):
            errors.append('Runtime data/configuration in Git: ' + name)
        if not path.is_file():
            errors.append('Tracked file is missing: ' + name)
            continue
        if path.stat().st_size > MAX_FILE_BYTES:
            errors.append('File exceeds the 12 MiB source/demo budget: ' + name)
        try:
            if path.suffix == '.md':
                check_links(name, files, errors)
            if path.suffix == '.json':
                json.loads(path.read_text(encoding='utf-8-sig'))
            if path.suffix in ('.launch', '.urdf') or path.name == 'package.xml':
                ET.parse(str(path))
            if path.suffix == '.py' and 'vendor' not in parts:
                ast.parse(path.read_bytes(), filename=name)
        except (ValueError, SyntaxError, UnicodeError, ET.ParseError) as exc:
            errors.append('{}: {}'.format(name, exc))
    if errors:
        for error in errors:
            print('ERROR: ' + error, file=sys.stderr)
        return 1
    print('Repository checks passed: {} tracked files, {} projects; entries, links, syntax and data exclusions.'.format(
        len(files), len(projects)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
