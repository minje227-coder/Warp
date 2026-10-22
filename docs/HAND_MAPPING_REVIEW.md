# 집게손 TCP 비교

`retarget.hand_review`는 원본 Quest RGB와 손목 중심의 3D 손 골격을 함께
보여 주는 오프라인 비교 도구다. 기본 retarget IK 설정은 변경하지 않는다.

```bash
cd /home/lab6/Warp
PYTHONPATH=pipeline/01_dataset .venv-retarget/bin/python -m retarget.hand_review \
  /home/lab6/Warp_data/2026-10-14/4b9f055a34a042af970f1b4f9c2764eb \
  local_data/retarget/hand-review-new-output \
  --config configs/kuavo_retarget.json --reference-start 5 --reference-end 7
```

`--frame-method mcp_pinch`를 추가하면 HumanEgo 식 (6)의 손목/엄지 MCP/
검지 MCP 좌표계를 비교한다. 기본 `palm_reference`는 손목/검지 MCP/약지
MCP 좌표계를 기준 구간의 기존 fingertip 방향과 맞춘다. 두 방식 모두
기본 IK에 반영하지 않는 독립 비교 도구다.

출력 디렉터리는 새 이름을 사용한다. 원본 파일 6개의 크기와 SHA-256을
검증하고 수정하지 않는다. 카메라 노출 순서와 프레임 수를 유지하며,
영상은 원본 간격에 따른 variable frame rate다. 손은 50ms 이내의 유효한
추적 샘플 사이에서만 보간하고, 무효 구간은 빈 패널로 표시한다.

색상:

- 주황: 엄지 TIP/검지 TIP 중간점 + MCP 기반 손바닥 방향.
- 하늘: 손바닥 좌표계에 고정한 가상 TCP + 같은 손바닥 방향.
- 분홍: 기존 검지 MCP→TIP 방향에서 나온 접근 방향.
- 노랑: 엄지 TIP와 검지 TIP를 잇는 선. 그리퍼 닫힘 축과의 각도를 표시한다.

손바닥 좌표계는 손목(1), 검지 MCP(7), 약지 MCP(17)에서 만든다.
접근/닫힘 축의 상수 회전과 가상 TCP 오프셋은 지정한 녹화 구간으로
추정한다. 위치는 손바닥 좌표계의 손목→집게 중간점 벡터의 성분별
중앙값, 회전은 기존 방향과 손바닥 방향 사이 상대 회전의 SO(3) 평균이다.
이는 시범 자세 기준이며 로봇 패드/접촉점 보정이 아니다. 그리퍼 형상은
추상 선 표시이며 실제 URDF 링크나 개폐 관절 동작을 재현하지 않는다.

`mcp_pinch`는 엄지 MCP(3)→검지 MCP(7)를 X축으로, 손목→두 MCP
중간점을 직교화한 벡터를 접근축으로 만든다. HumanEgo의 [X닫힘, Y접근,
Z법선]을 기존 손 좌표계 [X닫힘, -Y법선, Z접근]으로 상수 회전하고,
config의 `hand_to_tcp_rotation`을 적용한다. 현 설정은 로봇 -Z를 접근
방향으로 대응시키며 X의 부호는 기존 표시 기준을 따른다.
이 축 대응은 임시 가정이며 실제 손가락 패드/URDF 축 보정이 필요하다.
방향에 fingertip를 사용하지 않지만 엄지의 opposition 움직임은 반영된다.
HumanEgo의 confidence/SG/EMA/전체 학습 파이프라인은 재현하지 않는다.

[RoboWheel Appendix C.1](https://arxiv.org/html/2512.02729v1#A3.SS1)은
fingertip 기반 pinch와 MCP 기반 whole-hand 구성을 구분한다. 이 도구의
주황/하늘 방식은 그 구분을 참고한 비교 후보이며 Algorithm 1/2와
동일하지 않다. [WARP](https://arxiv.org/html/2606.29940v1#S3)은 palm
pose와 SEW를 맞추지만 KUAVO 두 손가락 접촉점 보정을 정해 주지 않는다.

[HumanEgo Appendix B.3](https://arxiv.org/html/2605.24934v1)은 위치를
엄지/검지 TIP 중간점, 방향을 손목과 엄지/검지 MCP의 Gram–Schmidt
좌표계, 개폐를 보정한 fingertip 거리로 분리한다. 공식 구현
`TX-Leo/HumanEgo`의 `preprocess/AriaHandsOptimizer.py`도 MCP 방향의
EMA와 재직교화/시간적 부호 일관성을 적용한다. 이번 비교는 핵심 좌표계만
독립 구현했다. 논문 및 소스 확인일은 2026-10-14이며 소스 commit은
`b7d9a43739681455dd3b9c0765e13433ce53e52c`이다.

## 관련 사례의 적용 범위

| 사례 | 핵심 방식 | 이번 상자 집기와의 관계 |
| --- | --- | --- |
| [HumanEgo B.3](https://arxiv.org/html/2605.24934v1) | TIP 중간점 위치, 손목/엄지 MCP/검지 MCP 방향, 보정한 TIP 거리 개폐 | 맨손 집게→parallel-jaw에 가장 직접적인 비교 후보. 전체 smoothing/학습/성공률을 KUAVO에서 재현한 것은 아니다. |
| [RoboWheel C.1](https://arxiv.org/html/2512.02729v1) | pinch와 whole-hand를 분리; pinch는 index MCP→TIP 방향, whole-hand는 wrist/index MCP/ring MCP | 기존 방향 구성의 근거지만 손가락 굽힘이 EE 회전에 섞이는지 확인해야 한다. 물체 움직임도 개폐 판단에 사용한다. |
| [YOTO IV-B](https://arxiv.org/html/2501.14208v1) | 손 위치/방향과 물체 접촉 개폐를 추출하고 keypose로 단순화해 IK 보간 | 양손 상자 작업 사례. 모든 원본 프레임을 그대로 연속 retarget한 방법과는 다르다. |
| [GAT-Grasp III-C/D](https://arxiv.org/html/2503.06227v1) | 엄지/검지 proximal과 index TIP의 방향을 grasp 생성의 조건으로 사용; 물체 접촉점/기하도 고려 | 손 방향뿐 아니라 물체의 잡을 부분과 실제 그리퍼의 가능한 grasp를 확인해야 한다는 참고. 연속 전체 시범 replay와는 다른 과제다. |
| [NVIDIA IsaacCapture](https://github.com/NVIDIA/IsaacCapture/blob/1641ca85642f582fa99d0e9f79722a116265dd0d/src/python/isaaccapture/retargeters/gripper_retargeter.py) | EE SE3와 그리퍼를 별도 retargeter로 분리; TIP 거리의 닫힘/열림 hysteresis | default 닫힘 30mm/열림 50mm는 teleop 제스처 기준이며 물체를 잡은 우리 손의 접촉/간격 기준으로 그대로 가져오면 안 된다. |
| [UMI](https://umi-gripper.github.io/umi.pdf) | 사람이 실제 parallel-jaw 수집 장치를 조작; TCP pose와 연속 jaw width를 측정 | 접촉 형상을 장치로 맞추는 접근. 맨손과 로봇 패드 사이 불일치를 줄이는 데 유용하지만 추가 장치가 필요하다. |

NVIDIA 소스 확인 commit은 `1641ca85642f582fa99d0e9f79722a116265dd0d`이다.
`se3_retargeter.py`는 wrist 위치/방향과 TIP 평균 위치/방향을 선택할 수
있으며, 개폐와 분리된 EE offset을 적용한다. WARP는 RB-Y1의 12-DoF
XHand를 사용하는 실험이므로 두 손가락 접촉 매핑의 직접 검증 사례로
간주하지 않는다.

`hand_targets.jsonl`에 후보 좌표, 원본 행 번호/추적 시간과 보간 비율을,
`reference.json`에 기준 자세와 사용한 추적 샘플을 보존한다.
`report.json`에는 원본/출력 해시, 유효율, 후보 차이, 조건부 비교를 기록한다.
조건부 비교는 약 0.25초 동안 손목 이동≤10mm, 손바닥 회전≤5°, 손끝 간격
변화≥8mm인 겹치는 관찰 구간이다. 실제 접촉 오차나 IK 실패 원인을
확정하는 독립 실험은 아니다. 손끝 간격이 10mm 미만이면 닫힘 축 각도는
정의하지 않는다.

RGB 화면 위의 정확한 투영은 하지 않는다. 카메라↔추적 시간은 callback
Unix/uptime의 중앙값으로 추정하고, 카메라↔world 외부 보정은 미검증이다.
3D 패널은 월드 축을 유지하고 손목 이동만 제거하며 전체 녹화에서 같은
시점/배율을 쓴다. 그리퍼 명령, 실제 로봇 동작, 학습 데이터 승인은 없다.

## RGB 손 위에 실제 그리퍼만 표시

`hand_overlay.py`는 원본 해상도 RGB에 URDF의 두 손가락 그리퍼 메쉬만
반투명하게 표시한다. 팔, 몸통, 손 골격과 비교 패널은 표시하지 않는다.
왼손은 하늘색, 오른손은 주황색이다. 메쉬는 TCP 기준의 실제 크기를
유지하며 표시 성능을 위해 단순화한다. 회전 관절은 URDF 기본 0으로
고정한다. 개폐 기구/패드 접촉/액추에이터 보정은 아직 하지 않는다.

입력 bundle은 원본 `camera.mp4`, `calibration.json`, `manifest.json`,
비교 도구의 `hand_targets.jsonl`, 원본 tracking에서 필터한 `head.jsonl`,
그리고 사용하는 `kuavo_s63_twofinger.urdf`로 구성한다. 원본 episode
폴더에 bundle을 추가하거나 원본 파일을 수정하지 않는다.

```bash
python pipeline/01_dataset/retarget/hand_overlay.py BUNDLE NEW_OUTPUT \
  --mesh-root /path/to/custom_urdf/leju_claw_two_finger/meshes \
  --hand-model /path/to/hand_landmarker.task
```

별도 Python 3.12 환경에 `requirements-hand-overlay.txt`를 설치한다.
MediaPipe의 공식 Hand Landmarker float16/1 모델을 사용한다. 기존
Drake/녹화/receiver 환경을 변경하지 않는다.

Camera2 `LENS_POSE_REFERENCE=1`은 gyro 기준이며, `LENS_POSE_ROTATION`은
Android 센서→카메라 회전이다. 따라서 inverse 회전을 사용해 optical
camera→head→Unity world→ROS world를 구성한다. head 위치는 보간하고
회전은 SLERP하며, invalid/outside/50ms 초과 head 구간은 그리퍼를
표시하지 않는다. `head_lines`는 필터한 `head.jsonl`의 행 번호다.

기록된 gyro 기준과 Unity head 원점 사이 관계는 검증되지 않았으므로,
영상의 손목/MCP 검출점으로 상수 6-DoF optical-frame 보정을 추정한다.
1초 시간 블록을 번갈아 학습/holdout으로 나누고 오차를 따로 기록한다.
MediaPipe는 보정 관찰점으로만 사용한다. 실제 표시하는 TCP 위치/방향은
Quest에서 기록한 MCP 후보이며 매 프레임 영상 손 위치로 snap하지 않는다.
이 보정은 해당 영상의 진단용 정합이며, 독립적인 headset 외부 보정이나
카메라/추적 시계 검증으로 인정하지 않는다. 영상 오차가 남을 수 있다.

출력은 `gripper-on-hands.mp4`, `projection-registration.json`,
`projection-frames.json`, `report.json`이다. report에는 입력과 메쉬의
SHA-256, 보정/holdout 오차, 표시 프레임 수와 개폐 고정 상태를 저장한다.
카메라 노출 간격대로 1프레임씩 출력하고 패딩하지 않는다.

근거: [Android CameraCharacteristics](https://developer.android.com/reference/android/hardware/camera2/CameraCharacteristics#LENS_POSE_ROTATION),
[CameraMetadata](https://developer.android.com/reference/android/hardware/camera2/CameraMetadata#LENS_POSE_REFERENCE_GYROSCOPE),
[MediaPipe Hand Landmarker](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python).

### 선택한 이진 개폐 미리보기

`--binary-gripper`는 엄지/검지 간격이 작을수록 커지는 닫힘값을 사용한다.
닫힘값≤0.35면 완전 열림, ≥0.65면 완전 닫힘이며 그 사이에서는 이전
상태를 유지한다. 추적 무효/긴 시간 단절 뒤에는 상태를 초기화하고,
처음부터 중간 영역이면 상태가 정해질 때까지 표시하지 않는다.
80ms 시간상수로 간격을 필터링하고 영상의 양손별 P20/P80을 임시
닫힘/열림 기준으로 사용한다. 물체 접촉/빈손/하드웨어 보정은 미검증이다.

`gripper-signals.json`은 원본 프레임, 닫힘값, 상태, 이진 열림 목표와
표시용 관절값을 보존한다. 몸/팔 IK는 변경하지 않으며, 열린 그리퍼의
충돌/스윕 검증이나 실제 액추에이터 명령이 아니다.

링크 각도는 lab6의 kuavo-ros-opensource
`MobileManipulatorDummyVisualization.cpp:getClawLinkageAngles`의
200062 표시 연동표를 참고한다. S63 custom URDF에는 bar-2와 finger
관절이 fixed이므로 이들은 회전시키지 않고, 존재하는 revolute인
bar-1/bar-3/bar-4만 좌우 대칭으로 적용한다. 관절 한계를 확인한다.
따라서 모델 표현은 실제 폐루프 기구의 정밀 보정을 대체하지 않는다.
