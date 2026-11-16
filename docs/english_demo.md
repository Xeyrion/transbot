# English demonstration entry

On the control PC, open **Transbot Data Workbench (EN)** from the desktop or application menu.

Command-line alternatives:

```bash
~/robot_research/bagtools/run_en.sh             # English workbench
~/robot_research/bagtools/run_en.sh --record    # English recording console
~/robot_research/bagtools/run_en.sh --review    # English session browser
```

The English entry sets `BAGTOOLS_LANG=en` for this process and its tools. It uses exactly the same code, data roots, session folders and recording logic as the Chinese entry. The original Chinese desktop entry remains available; changing the UI language does not rename any files or modify a recording.

The workbench, recording console, session list, quality details, bag viewer, loading indicators and dialogs use the shared `locale/en.json` catalogue. Recorder status and error messages are translated too — `i18n.translate_log()` handles them in three passes: exact catalogue hit, catalogue hit after stripping indentation, then a regex table for lines that carry paths, PIDs or capacities. File paths, topic names and unknown raw diagnostics are preserved verbatim; anything unrecognised stays in the original wording rather than being dropped.

Recorder scripts deliberately keep their Chinese source text. They run on the robot behind ssh, where environment variables do not follow, so translation happens once, in the display layer. Two boundaries are intentional: the `-h` help text of `remote_record.sh` / `record_full.sh` / `auto_record_cam.sh` stays Chinese (developer help, not part of the demo), and the `docs/` folder stays Chinese.

`tests/test_i18n.py` walks every `echo`/`print` literal in the recorder scripts and fails if any Chinese survives translation, so a newly added message cannot silently ship as Chinese-only.

Chinese entry: `run.sh` or **Transbot 数据工作台**. For scripts, `BAGTOOLS_LANG=zh` / `en` explicitly selects a language before starting the program; live switching inside an already-open window is intentionally not required.
