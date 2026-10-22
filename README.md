# Warp

Meta Quest의 RGB·인체·손 움직임을 수집하고, WiLoR/Meta 융합과 물체 추정을 거쳐
KUAVO S63의 로봇 관절·그리퍼 궤적으로 변환하는 프로젝트입니다.

```text
Quest 원본 수집 → WiLoR/Meta 융합 → 태그·world 준비
→ KUAVO IK → constrained C2 → TOPPRA 시간 재조정 → 검증·4분할 영상
→ 로봇 데이터셋·학습·RL (후속 단계)
```

## Retargeting standard — 2026-10-22 채택

WiLoR→KUAVO 계산·검증·4분할 영상은 채택된 **wilor-kuavo-standard**를 사용합니다.
날짜별 실험 solver/renderer를 자동 선택하지 않습니다.

- 랙 태그 중심 높이: **0·4·1 = 1.615m**, **2·5·3 = 1.000m**. 녹화 좌표의 공통 Z 이동으로 맞추며, 로봇 카메라·TF로 높이를 다시 계산하지 않습니다. 녹화 XY·yaw·형상은 유지하고 태그별 잔차를 기록합니다.
- 양손 TCP: 기본 **3cm**, 각 박스 태그의 **-Y 아래축**으로 전체 궤적에 적용합니다. 고정 좌표계 수직 보정·대체 경로는 없습니다. 원본 입력에서 매번 계산하므로 누적되지 않습니다.
- 그리퍼: 보간한 열림 값 **40% 이하 → 완전히 닫기**. 초과 값은 유지하며 첫 자세·재생·영상에 동일하게 적용합니다. 힘·속도를 높이는 설정은 아닙니다.
- 영상: 닫힘 구간에는 **그리퍼 손가락 접촉부만 빨간색**, CLOSE 문구 없음. 우하단은 **측면 뷰**, 그 창의 사람 스켈레톤은 숨깁니다. 빨간색은 닫힘 명령 표시이며 실제 접촉 검출은 아닙니다.
- 실시간 head 영상은 대시보드와 **동일한 최신 버퍼/엔드포인트**를 사용합니다. 태그 실패로 영상이 멈추지 않습니다. RWH 태그 하나로도 박스를 투영하지만, 로봇 정렬·이동 허용에는 사용하지 않습니다.

```bash
cd /home/lab6/Warp
bash scripts/02_convert/convert_episode_standard.sh /path/to/episode --prepared /path/to/prepared.json
```

녹화별 입력 준비 후 **native IK → constrained C2 B-spline → TOPPRA → dense/MoveIt 검증 → 4-window 렌더링**을 수행합니다.
대시보드 계산은 고정 높이·박스축 보정을 준비한 뒤 이 진입점을 호출합니다. CLI에 원본 prepared를 직접 넘기면 UI 전처리를 자동 적용하지 않으므로 아래 문서를 따르세요.
기존 lazy base·몸통/양팔·고정 head와 공통 시간축을 유지하며 Kalman/jerk는 사용하지 않습니다.
과거 **13.35초는 준비된 입력의 IK+시간 재조정만**이며 전체 처리 시간이 아닙니다.

[실행·입력 준비·전체 합의](docs/RETARGETING_STANDARD.md) · [검증·실패 기록](docs/RETARGETING_VALIDATION.md) · [채택 코드/정책](pipeline/01_dataset/kuavo_standard) · [대시보드 설치/재생](scripts/05_execute/README.md)

## 빠른 시작

아래 명령은 환경과 수신 서비스가 설치된 **lab6** 기준입니다.
처음 설치할 때는 [설정 안내](docs/SETUP_OPTIONS.md)와 [raw v3 수집 안내](docs/RAW_CAPTURE.md)를 참고하세요.

### 1. 수집

```bash
cd /home/lab6/Warp
systemctl --user start warp-raw-receiver warp-raw-processing
```

Quest 앱에서 **볼륨 업**으로 녹화를 시작·종료하고, 대기 중 **볼륨 다운**으로 HEAD/NECK을 전환합니다.
원본은 로컬 저장 후 PC로 비동기 전송됩니다. 기본 카메라 요청은 1280×1280·30fps입니다.
서비스 설치, 데이터 형식과 전송 복구는 [수집 안내](docs/RAW_CAPTURE.md)에 정리되어 있습니다.

### 2. 녹화 변환

해당 녹화의 `prepared.json`과 참조 궤적·world·설정을 먼저 준비합니다.
WiLoR 추론·Meta 융합·태그 추정은 별도 전처리이며, 아래 명령은 준비된 입력부터 실행합니다.

```bash
# 입력과 실행 명령 확인
bash scripts/02_convert/convert_episode_standard.sh /path/to/episode --dry-run

# 채택 standard로 IK → 시간 재조정 → 검증 → 4분할 렌더링
bash scripts/02_convert/convert_episode_standard.sh /path/to/episode
```

**WiLoR→KUAVO 계산·검증·렌더링은 채택된 standard를 사용합니다.**
실험 폴더를 실행 경로로 대체하지 않습니다. [입력 준비와 실행 계약](docs/RETARGETING_STANDARD.md)을 참고하세요.

### 3. 채택 결과 확인

```bash
runtime=/home/lab6/Warp-review/wilor-kuavo-standard
/home/lab6/Warp/.venv-retarget/bin/python "$runtime/standard.py" status
/home/lab6/Warp/.venv-retarget/bin/python "$runtime/standard.py" verify TAKE
/home/lab6/Warp/.venv-retarget/bin/python "$runtime/standard.py" render TAKE
```

`status`에서 `CURRENT.json`이 선택한 TAKE를 확인합니다. 새 녹화의 실행 결과는 자동으로 CURRENT를 바꾸지 않습니다.

## 입력·출력 위치

| 항목 | 위치·내용 |
|---|---|
| 녹화 원본 | `/home/lab6/Warp_data/YYYY-MM-DD/TAKE/`: RGB, tracking, 시각·보정 정보 |
| 원본 전처리 | 녹화 폴더의 `processed/`: 태그 검출, 시간 대응, 품질 검사 |
| 변환 입력 | runtime의 `prepared/TAKE/prepared.json` 또는 `--prepared`로 지정한 해당 녹화의 bundle |
| 변환 결과 | runtime의 `results/RUN_NAME/`: 궤적, 검증 보고서, 4분할 영상 |
| 채택 소스·설정 | [`pipeline/01_dataset/kuavo_standard/`](pipeline/01_dataset/kuavo_standard) |

녹화 원본과 생성 영상·궤적은 Git에 포함하지 않습니다.

## 현재 상태

| 구간 | 현재 범위 |
|---|---|
| Quest raw v3 수집 | 독립 RGB·tracking 저장, 비동기 전송, PC 전처리 구현 |
| 입력 준비 | WiLoR/Meta 융합, 태그·world와 녹화별 reference를 별도로 준비 |
| KUAVO 변환 | 채택 IK·TOPPRA·충돌/경로 검증·4분할 렌더링 구현; 준비된 녹화에서 offline 검증 |
| 물체 추정 | 랙·박스 공동 태그 추정 및 현재 100·101 박스의 승인된 상대 배치 보정 |
| 데이터셋·학습·RL | 위 변환 경로 이후의 후속 단계 |

현재 검증 범위와 측정 결과는 [검증 기록](docs/RETARGETING_VALIDATION.md)에 있습니다.
수집 파일 검사, offline 궤적 검증과 실제 로봇 실행의 성공은 각각 구분합니다.

## 문서

- [수집·전처리](docs/RAW_CAPTURE.md)
- [Retargeting 실행·입력·현재 정책](docs/RETARGETING_STANDARD.md)
- [검증 결과·시간 측정](docs/RETARGETING_VALIDATION.md)
- [설치·환경 선택](docs/SETUP_OPTIONS.md)
- [Upstream·revision·라이선스](docs/UPSTREAMS.md)
- [이전 수집 방식·로드맵 보존본](docs/history/README_BEFORE_CLEANUP_20261019.md)
