import ast
import os
from pathlib import Path
import re
import sys
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from i18n import tr,english_catalog,translate_log,untranslated
from workbench import launch_spec

CJK=re.compile(u'[\u4e00-\u9fff]')

# 录制脚本的中文输出：GUI 日志里全部要翻成英文。脚本本身不做 i18n（机器人在 ssh
# 那头跑，环境变量过不去），翻译只在显示层做，所以这里守着显示层的完整性。
SHELL_SCRIPTS=['recording/remote_record.sh','recording/record_full.sh',
               'recording/auto_record_cam.sh','recording/restart_camera.sh',
               'recording/deploy_to_robot.sh','recording/run.sh',
               'recording/preflight.sh']
PY_SCRIPTS=['recording/stop_recording.py','recording/session_status.py']


def _skip_cmdsub(text,pos):
    """pos 指向 '$('，返回配对的 ')' 之后的位置。命令替换里常有引号和嵌套括号。"""
    depth=0
    while pos<len(text):
        if text[pos]=='(':depth+=1
        elif text[pos]==')':
            depth-=1
            if depth==0:return pos+1
        pos+=1
    return len(text)


def _render(text):
    """把源码里的字面量变成脚本运行时大概会打出来的样子：变量换成 X。"""
    text=text.replace('\\$','\x00')
    out=[];i=0
    while i<len(text):                      # $(命令替换)，要按括号配对吃掉
        if text.startswith('$(',i):
            out.append('X');i=_skip_cmdsub(text,i);continue
        out.append(text[i]);i+=1
    text=''.join(out)
    text=re.sub(r'\$\{[^}]*\}','X',text)          # ${VAR}
    text=re.sub(r'\$[A-Za-z_][A-Za-z0-9_]*','X',text)   # $VAR
    text=re.sub(r'%[-+ #0-9.]*[diouxXeEfgGrsc]','X',text)  # %s / %.1f
    return text.replace('\x00','$')


def _literal_after(line,pos):
    """取出命令词后面的那个字符串字面量（没有引号就取到重定向/行尾）。"""
    rest=line[pos:]
    quote=rest[:1]
    if quote in ('"',"'"):
        out=[];i=1
        while i<len(rest):
            ch=rest[i]
            if ch=='\\' and i+1<len(rest):out.append(rest[i+1]);i+=2;continue
            if ch=='$' and rest.startswith('$(',i):
                end=_skip_cmdsub(rest,i);out.append(rest[i:end]);i=end;continue
            if ch==quote:break
            out.append(ch);i+=1
        return ''.join(out)
    return re.split(r'\s*[12]?>&[12]|\s*\|\s*|\s*;\s*',rest)[0].strip()


def shell_messages():
    out=[]
    for name in SHELL_SCRIPTS:
        for n,line in enumerate((ROOT/name).read_text(encoding='utf-8').splitlines(),1):
            # 整行注释不执行。注释里出现 "rostopic echo ..." 会被下面那个正则
            # 当成真的 echo 语句，报一条根本不存在的未翻译串。
            if line.lstrip().startswith('#'):
                continue
            for m in re.finditer(r'\b(echo|printf)\s+',line):
                text=_literal_after(line,m.end())
                if text and CJK.search(text):out.append((name,n,_render(text)))
    return out


def _const_str(node):
    """字符串字面量。3.8 起是 ast.Constant，之前是 ast.Str —— 两边都要认，
    否则这些测试在新解释器上会静默变成空循环。"""
    if isinstance(node,getattr(ast,'Constant',())) and isinstance(node.value,str):
        return node.value
    if isinstance(node,ast.Str):
        return node.s
    return None


def python_messages():
    out=[]
    for name in PY_SCRIPTS:
        tree=ast.parse((ROOT/name).read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if not (isinstance(node,ast.Call) and isinstance(node.func,ast.Name)
                    and node.func.id=='print' and node.args):continue
            arg=node.args[0]
            if isinstance(arg,ast.BinOp):arg=arg.left     # '前缀：' + str(x)
            text=_const_str(arg)
            if text:
                text=_render(text)
                if CJK.search(text):out.append((name,node.lineno,text))
    return out


class TranslationTests(unittest.TestCase):
    def test_format_contracts(self):
        pattern=r'%(?:[-+#0-9.]*[sdf]|%)'
        for source,target in english_catalog().items():
            self.assertEqual(re.findall(pattern,source),re.findall(pattern,target),source)

    def test_ui_catalog_is_complete(self):
        catalog=english_catalog()
        found=0
        for name in ['workbench.py','bag_quality_gui.py','recording/record_gui.py','session_catalog.py','bag_viewer_server.py']:
            tree=ast.parse((ROOT/name).read_text(encoding='utf-8'))
            for node in ast.walk(tree):
                if isinstance(node,ast.Call) and isinstance(node.func,ast.Name) and node.func.id=='tr':
                    key=_const_str(node.args[0]) if node.args else None
                    if key is not None:
                        found+=1
                        self.assertIn(key,catalog,name)
        self.assertGreater(found,150,'tr() 扫描失效了：一条都没扫到')

    def test_script_output_is_translated(self):
        """录制脚本的每一句中文输出，在英文模式下都不能残留中文。"""
        shell=shell_messages()
        py=python_messages()
        # 抽取器自己也要被测：抽不到东西就等于没测
        self.assertGreater(len(shell),60,'shell 消息抽取失效')
        self.assertGreater(len(py),4,'python 消息抽取失效')
        with patch.dict(os.environ,{'BAGTOOLS_LANG':'en'}):
            for name,line,text in shell+py:
                self.assertEqual(untranslated(text),[],'%s:%d %r'%(name,line,text))

    def test_paths_and_diagnostics_survive(self):
        """翻译只动措辞：路径、PID、容量这些诊断信息必须原样留下。"""
        with patch.dict(os.environ,{'BAGTOOLS_LANG':'en'}):
            cases=[
                ('本次会话目录：/home/jetson/bags/session_20261111_0143_x',
                 'Session directory: /home/jetson/bags/session_20261111_0143_x'),
                ('保存目录 : /home/jetson/bags（剩余 11 GB）',
                 'Output directory: /home/jetson/bags (11 GB free)'),
                ('ERROR: 已经有一个录制会话在跑（PID 4321），先停掉再开：',
                 'ERROR: a recording session is already running (PID 4321); stop it first:'),
                ('=== 会话收尾：完成 3 卷，未关闭 0 卷 ===',
                 '=== Finalized: 3 parts; unfinished: 0 ==='),
                ('  /scan 有数据','  /scan is streaming'),
                ('某个没人认识的报错 Traceback','某个没人认识的报错 Traceback'),
            ]
            for source,want in cases:
                self.assertEqual(translate_log(source),want)

    def test_default_chinese_and_english_inheritance(self):
        with patch.dict(os.environ,{'BAGTOOLS_LANG':'zh'}):
            self.assertEqual(tr('录制控制'),'录制控制')
            self.assertEqual(translate_log('本次会话目录：/tmp/x'),'本次会话目录：/tmp/x')
        with patch.dict(os.environ,{'BAGTOOLS_LANG':'en'}):
            self.assertEqual(tr('录制控制'),'Recording Console')
            self.assertEqual(launch_spec('record','/tmp/data')[1]['BAGTOOLS_LANG'],'en')
            self.assertEqual(launch_spec('review','/tmp/data')[1]['BAGTOOLS_LANG'],'en')
            self.assertEqual(translate_log('本次会话目录：/tmp/session_x'),'Session directory: /tmp/session_x')


if __name__=='__main__':unittest.main()
