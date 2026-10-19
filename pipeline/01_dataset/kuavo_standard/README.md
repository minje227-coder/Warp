# Adopted WiLoR–KUAVO standard

**MUST use the adopted standard for computation and all four rendered windows.**
Do not select dated experiments as the active pipeline.

See [the run guide](../../../docs/RETARGETING_STANDARD.md) and
[the episode-path bash command](../../../scripts/02_convert/convert_episode_standard.sh).
This directory freezes implementation/configuration; recordings/results remain in the lab6 runtime.

# WiLoR → KUAVO standard (2026-10-17)

현재 채택: 기존 lazy base 억제 + native Drake/SNOPT IK + 팔/몸통/rack/1mm open-box shell 충돌 + TOPPRA 공식 속도/가속도.
9~16초 고정이나 녹화별 시간 조건은 없다. 기존 base reference를 그대로 사용한다.
그리퍼 8개 충돌 형상은 제외하고 손목 link7은 유지한다. 충돌 여유 10mm.
TCP 목표는 원래 grasp point(world offset 0). 실제 box 옆면/밑면 거리 10cm 이내 허용오차 10mm,
20cm 이상 20mm, 사이 연속 전환. 손 방향 제약과 양손 평행 10도는 유지한다.
몸통 연속성 비용 1000, 팔 100, 예측 연속성 비용 300; 가중치는 물리적 한계가 아니다.
SNOPT 내부 위치/충돌 reserve는 검증 상한을 바꾸지 않는다.
고립된 입력 이상값은 일반 검출 규칙으로 앞뒤 timestamp 보간한다. 원본 녹화는 변경하지 않는다.
공식 S63 속도/가속도 적용, jerk 및 low-level Kalman 미적용.

`standard.py status`, `standard.py verify <take>`로 현재 채택 결과를 확인한다.
`standard.py render <take>`는 해당 결과의 4분할 영상을 확인/생성한다.
`standard.py run <take> --reference <prepared trajectory.jsonl> --world <world.json> --reference-config <config.json>`
으로 같은 설정의 IK/retime/dense check/MoveIt 검증을 수행한다. `--display-source <take-specific rendering snapshot>`은 선택이다.
현재 run 인터페이스는 원본 녹화 직접 변환이 아니라 녹화별 준비된 입력을 받는다.
원본 WiLoR/Meta 융합, bounded missing input 보간, 태그 추정 및 lazy base reference 생성은 별도 전처리다.
다른 녹화의 초기 관절이나 물체 위치를 자동 재사용하지 않는다.
입력이 있는데 기존 reference IK만 실패한 프레임은 take별 seed/base를 명시해 준비해야 하며,
누락 입력과 IK 실패를 혼동해 유효 프레임으로 꾸미지 않는다. 실행 출력은 전 입력 성공 후에만 생성한다.
중간 자세 검증과 MoveIt이 통과해야 검증 결과로 수락한다. 기존 결과 덮어쓰기는 거부한다.

현재 결과: 새 녹화 0~20초, 유효 597개 및 중간 6257개 자세 검증, MoveIt 실패 0.
실행 시간 10.092초. 실제 로봇으로 전송하지 않았으며 초기 실기 자세 연결은 별도다.
과거 standard 전체는 `../backups/standard-before-lazybase-collision-toppra-20261017`에 보존했다.
기존 태그 정책/tag JSON은 유지하며, 녹화별로 그 시점의 태그 배치를 사용한다.

## 랙 태그 배치 (2026-10-19 채택)

랙 정면 기준 위쪽 선반은 `0 — 4 — 1`, 중간 선반은 `2 — 5 — 3`.
4는 0·1 중심 사이의 정중앙, 5는 2·3 중심 사이의 정중앙이며 각 줄은 같은 높이와 방향이다.
태그는 tag36h11, 검은 외곽 한 변 40mm, 사방 흰 여백 5mm, 종이 전체 50mm.
양끝 종이 옆변은 기둥 안쪽 경계에 맞추고, 태그 중심은 선반 앞면 두께 중앙에 맞춘다.
좌우 끝 중심 간격 760mm, 중앙과 양끝 간격 각각 380mm, 두 줄 높이 차이 610mm.
geometry tag 좌표계에서 4의 중심은 `[0, 0, 0.05]`m, 5는 `[0, -0.61, 0.05]`m.
기존 0~3의 위치와 geometry 기준은 유지한다. 새 배치는 이후 녹화의 태그/world 준비에 적용하며,
과거 녹화의 prepared world와 검증 결과는 당시 배치를 그대로 사용한다.

## 랙·박스 공동 태그 자세 추정 (2026-10-19)

새 녹화의 world 준비는 `tools/build_tag_world.py`와 `tools/multi_tag_pose.py`를 사용한다.
랙은 0~5, 현재 A13 박스는 기존 좌상단 100·우상단 101의 알려진 위치를 사용한다.
한 개만 보이면 기존 IPPE_SQUARE를 사용하고, 두 개 이상이면 보이는 태그의 코너를
공통 물체 좌표계에 모아 공동 planar PnP/RANSAC과 LM 재최적화를 수행한다.
태그 단위로 이상치를 제외하며 공동 결과는 온전한 태그 두 개 이상을 설명해야 한다.
태그별 RMS 2px 및 코너 3px 기준을 통과하지 못하면 기존 최선 단일 추정으로 복귀하되
`single_tag_fallback`과 실패 이유를 기록한다. 이를 공동 추정 성공으로 집계하지 않는다.
원래 관측 ID, 사용/제외 ID, 사용 코너 수, 태그별 오차를 프레임별로 남긴다.
랙은 선택 구간에서 고정 자세 하나로 통합하고, 박스는 기존 ±0.3초 평균을 유지한다.
기존 부착 치수, 원본 영상/타임스탬프, prepared world, CURRENT 결과와 IK 정책은 보존한다.

동일한 기존 0~20초 입력으로 world 생성 검증: 랙의 다중 검출 115프레임 중 114프레임에서
공동 추정 통과, 1프레임은 명시적 단일 복귀. 박스의 다중 검출 486프레임은 모두 공동
오차 기준을 통과하지 못해 단일 복귀했으며 박스 위치 결과는 기존과 같다.
이는 박스 공동 추정의 실영상 정확도 향상을 입증한 결과가 아니다. 기존 좌상단/우상단
배치를 유지한 채 부착 기하와 관측의 불일치 원인을 별도로 확인해야 한다.
합성 검증은 랙 63개 부분집합, 박스 공동 8코너, 3태그 12코너, 이상 태그/코너 제외,
단일 추정 호환성 및 잡음 조건을 포함한다. 실제 박스 기준으로 치수를 임의 보정하지 않는다.

태그/world 준비에는 OpenCV가 추가로 필요하다. lab6에서는 기존 box_detection의 cv2와
`.venv-retarget`의 NumPy/SciPy를 사용하며, IK 실행 환경을 새로 설치하거나 변경하지 않는다.
원본 전처리와 IK 실행의 경계는 그대로다. `--root`는 별도의 새 world 출력 디렉터리로 지정한다.
