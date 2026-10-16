# Repository instructions

## WiLoR → KUAVO retargeting

**MUST use the adopted standard** for retargeting computation, collision verification,
time assignment and four-window rendering. Read `docs/RETARGETING_STANDARD.md` first.
Use `scripts/02_convert/convert_episode_standard.sh EPISODE_PATH`, which calls the
adopted lab6 `wilor-kuavo-standard/standard.py` runtime and its configuration.
Do not automatically select date-stamped experiment directories or substitute a
separate IK solver/renderer. Prepared references may originate from documented
experiments, but the active computation policy comes from the adopted standard.
The frozen implementation lives in `pipeline/01_dataset/kuavo_standard`.
`CURRENT.json` identifies the adopted validated result. New run outputs do not
silently replace it. Keep the existing standard unless the user explicitly adopts
a change. Preserve raw recordings and separate input preparation from the measured
13.35s IK+timing subtotal. Preserve the four panels: external RGB, My view,
whole-body+Kuavo, robot head view. Never hide IK error using display transforms.

These requirements apply to WiLoR/KUAVO retargeting, not unrelated capture,
training or reinforcement-learning code.
