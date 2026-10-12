# Setup menu: Linux / macOS / Windows and selectable capture options

**Goal:** preserve portability and prevent hardware assumptions in the repository. Setup only records choices; it does not mean the 3-stage research pipeline has been implemented.

## 1. Platform (one host at a time)

| Option | Run script in | Notes |
| --- | --- | --- |
| `linux` | Terminal / Bash | Check Python, Git and XR dependencies; Unity+Quest compatibility must be tested for selected version. |
| `macos` | Terminal / Bash | Same Python/Git structure; Android/Unity SDK path differs. |
| `windows` | **Git Bash** | Bash scripts are not native PowerShell scripts; Quest USB, Unity/Android tooling need setup. |
| `auto` | Detect current Bash environment | If running under **WSL**, the environment is Linux, select `linux`. |

`--os` only changes the recorded host target and instructions; it does not install packages, build a Quest APK or enable camera permissions. It is possible to prepare a Windows setting on a Linux computer, but the installer will warn that host and target differ.

## 2. Other choices

- Quest: `quest3`, `quest3s`, `questpro`, `quest2`, `other`, `none`. **Body Tracking and Passthrough Camera API support differ across models and firmware.** XRT-Client upstream specifically documents Quest 3; do not assume full functionality elsewhere.
- Capture streams (multi-select): `body`, `hand`, `rgb_left`, `rgb_right`; `all`, `none`. Default recorder captures body/hand + one physical Quest RGB eye (right by default), with original timestamps and sequence IDs. A selection does not enable both cameras.
- Sources (multi-select): `capture`, `video`, `retarget`, `gripper`; `all`, `none`. Actual source retrieval is opt-in; run `bootstrap.sh ...` or select **download** in the menu. This is independent of captured streams.

### Interactive

```bash
bash scripts/setup.sh
```

### Noninteractive examples

```bash
bash scripts/setup.sh --os linux --quest quest3 --streams all --modules capture,video
bash scripts/setup.sh --os macos --quest quest3 --streams hand,rgb_left --modules all
bash scripts/setup.sh --os windows --quest quest3s --streams body,hand --modules capture
bash scripts/setup.sh --os auto --quest other --streams none --modules none
```

### Preview without changes

```bash
bash scripts/setup.sh --os windows --quest quest3 --streams all --modules all --dry-run
bash scripts/bootstrap.sh --dry-run capture video retarget
```

## 3. Safety / maturity / data paths

Settings live in `configs/local.env` and are **ignored by Git**. Human pose, faces, videos, and recordings belong in ignored `local_data/`; none of them belong in tracked source control. This selection does not clone code by default; with `--download` it only retrieves **pinned Git submodules**, subject to public upstream availability and their own licenses.

`capture` refers to Quest Body/Hand streaming and CSV recording examples. RGB is recorded by the separate Camera2 helper and required by the recorder by default. The `video` upstream remains reference material. Hardware acquisition sync/world calibration, retargeting for the 4-DoF waist, gripper mapping, collision-aware validation, policy training, and RL fine-tuning are **not yet implemented**. Confirm device support and research licenses before deploying.
