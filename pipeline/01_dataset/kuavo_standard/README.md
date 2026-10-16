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
