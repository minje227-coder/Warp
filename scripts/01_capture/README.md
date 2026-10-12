# Capture scripts

- `install_receiver.sh` — Python 3.10+ `.venv`를 만들고 `XRT_devices[xr,recording]` 설치. 먼저 `bash scripts/bootstrap.sh capture` 필요.
- `validate_body.sh` — Python 실행 이름과 관계없이 CSV 무결성/SEW 검사.
- `record_body.sh` — Quest Body/Hand Bone CSV 수집. `--session`, `--port`, `--duration`, `--wait-seconds`, `--check`를 지원.

```bash
bash scripts/bootstrap.sh capture
bash scripts/01_capture/install_receiver.sh
bash scripts/01_capture/record_body.sh --check
bash scripts/01_capture/record_body.sh --session smoke_0001 --duration 20
bash scripts/01_capture/validate_body.sh local_data/raw_human/smoke_0001 --strict-sew
```

**아직 없는 기능:** Quest Egocentric RGB + Body 동시 저장, Frame Capture Timestamp Sync, AprilTag/Box Pose, Robot Retargeting, 실제 장비 테스트. 자세한 내용은 [Capture Guide](../../docs/CAPTURE_GUIDE.md).
