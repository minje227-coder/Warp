# Dataset Stage (현재 최우선)

## 현재 동작하는 최초 기능

- `record_quest_body.py`: XRT-Client → XRT_devices Receiver → Human Skeleton CSV 및 JSON 메타데이터. **Quest 장비 미테스트.**
- `validate_csv.py`: Bone CSV 형태·프레임·SEW Landmark Coverage·Gap 확인. **Synthetic Test 통과.**
- 실행 래퍼: `bash scripts/01_capture/record_body.sh --session smoke_0001 --duration 20`
- Receiver 설치: `bash scripts/01_capture/install_receiver.sh`

이 단계는 **Human Pose 파일만 수집**한다. Gripper Action, Robot Joint, Box Pose, RGB, Calibration, Retargeting, 학습 데이터 변환은 아직 미구현이다.

[처음부터 따라 하는 수집 절차](../../docs/CAPTURE_GUIDE.md)

## 향후 단계

1. Synchronized RGB/Body/Hand 기록, Camera Intrinsics/Extrinsics 보존
2. Human/Rack/Object 공통 좌표계 및 Grasp State Estimation
3. SEW Base/Waist/Arms → Gripper TCP Retargeting
4. Collision/Grasp/Lift 재생 검증
5. LeRobot Robot Observation/Action 데이터셋으로 변환
