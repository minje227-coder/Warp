# WiLoR → KUAVO adopted standard

WiLoR/Meta의 녹화별 준비된 입력을 KUAVO S63 궤적으로 변환하는 채택 소스·설정입니다.
**계산·검증·4분할 렌더링은 이 standard를 사용합니다.**

## 실행

lab6의 `/home/lab6/Warp`에서 해당 녹화의 입력을 준비한 뒤 실행합니다.

```bash
bash scripts/02_convert/convert_episode_standard.sh /path/to/episode --dry-run
bash scripts/02_convert/convert_episode_standard.sh /path/to/episode
```

입력은 해당 episode의 `prepared.json`, reference, world, reference config와 rendering provider입니다.
원본 WiLoR·Meta·태그 전처리는 별도 단계입니다. 결과는 lab6 runtime의 `results/RUN_NAME/`에 저장됩니다.
`CURRENT.json`은 채택된 검증 결과를 선택하며 새 실행으로 자동 교체되지 않습니다.

## 파일 안내

| 파일 | 역할 |
|---|---|
| [standard.py](standard.py) | `status`, `run`, `verify`, `render` 진입점 |
| [STANDARD.json](STANDARD.json), [standard_config.json](standard_config.json) | 채택 solver profile·IK 설정 |
| [motion_policy.json](motion_policy.json) | 시간 재조정·속도·가속도 정책 |
| [object_overlay_config.json](object_overlay_config.json), [tag_policy.json](tag_policy.json) | 랙·박스 추정과 태그 배치 |
| [box_tag_calibration.json](box_tag_calibration.json) | 승인된 고정 박스 태그 보정·출처 |
| [tools/build_tag_world.py](tools/build_tag_world.py) | 해당 녹화의 태그 관측으로 world 준비 |
| [SNAPSHOT_SHA256.json](SNAPSHOT_SHA256.json) | 소스·설정 파일 해시 |

녹화·생성 궤적·영상은 이 소스 디렉터리에 포함하지 않습니다.

## 상세 문서

- [실행 방법·입력 계약·현재 정책](../../../docs/RETARGETING_STANDARD.md)
- [검증 결과·측정 범위](../../../docs/RETARGETING_VALIDATION.md)
- [프로젝트 시작 안내](../../../README.md)
