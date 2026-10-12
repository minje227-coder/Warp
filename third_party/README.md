# Upstream code (Git submodules)

All third-party source trees are pinned Git submodules (Git mode 160000); they are not vendored and are not present until downloaded.

```bash
bash scripts/bootstrap.sh capture   # Python XR receiver + Quest client
bash scripts/bootstrap.sh video     # Meta passthrough RGB camera sample
bash scripts/bootstrap.sh retarget  # RBY1 teleop / MATLAB SEW and IK references
bash scripts/bootstrap.sh gripper   # RoboWheel toolkit; not the complete retargeter
```

Do not commit upstream code copies or modify source inside submodules by accident. For project-owned adapters use `pipeline/01_dataset/`.

See `docs/UPSTREAMS.md` and `docs/STATUS.md`.
