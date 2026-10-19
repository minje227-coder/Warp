# Retargeting 검증 기록

이 문서는 특정 입력·시점의 검증과 측정 결과를 보존합니다.
현재 실행 방법과 정책은 [standard 안내](RETARGETING_STANDARD.md)를 따릅니다.

## 2026-10-17: 준비된 입력의 offline retargeting


For the adopted 20-second demo, using already prepared input:

| Stage | Wall time |
|---|---:|
| Native IK | **11.72s** |
| TOPPRA timing | **1.63s** |
| IK + timing subtotal | **13.35s** |
| Dense Drake verification | **45.55s** |
| MoveIt verification | Additional; not separately timed |
| Four-window rendering | **16.87s** |
| Duration of the resulting motion | **10.09s** — playback time, not calculation time |

**13.35s excludes WiLoR inference, input fusion, tag/world preparation, validation
and video rendering.** Existing saved trajectories do not need IK again for replay.
The robot has not physically executed this offline result.

Verification: **597 valid knots**, **6,257 dense states**, MoveIt collision/bounds
failures **0**; the standard entry point reproduced all 600 source rows' validity,
base and joint solutions exactly. Minimum sampled clearance **10.89mm**. The first
three observations were unavailable; 597 is not a claim of 600 solved inputs.

Other recording: 417 prepared states solved, no MoveIt collisions; 70 dense states
at source 14.18–14.48s exceeded TCP allowance. That result was rejected, not promoted.
Its 23 missing/failed-reference rows and 10 unobserved-box rows were outside that
prepared-input test. This is not a full success claim for arbitrary recordings.


## 2026-10-19: 공동 태그 추정 도입, 박스 보정 전

같은 0~20초 입력에서 랙의 다중 검출 115프레임 중 114프레임이 공동 추정을 통과했고,
1프레임은 단일 추정으로 복귀했습니다. 박스는 당시의 19.5cm·같은 높이·같은 방향 가정으로
공동 기준을 통과하지 못해, 다중 검출 486프레임 모두 단일 추정으로 복귀했습니다.
이 배치는 아래 승인된 보정으로 대체되었습니다.

## 2026-10-19: 박스 태그 보정 채택

현재 박스 100·101의 상대 배치를 영상에서 추정하고, 실제 원본에 투영한 비교를 검토한 뒤 채택했습니다.
사용한 고정 후보는 0~7초의 60프레임에서 추정한 `early_to_late`입니다.
별도 13~20.1초 구간에서는 148프레임 중 147프레임이 픽셀 기준을 통과했습니다.
이 값은 영상 기반 상대 태그 보정이며 물리 치수의 실측이나 카메라 보정을 대체하지 않습니다.

| 같은 0~20초 입력의 world 생성 | 결과 |
|---|---:|
| 입력 paired frames | 582 |
| 박스 관측 프레임 | 580 |
| 두 태그 검출 중 공동 추정 통과 | 484 / 486 |
| 공동 추정 실패 후 단일 복귀 | 2 |
| 한 태그만 검출되어 단일 추정 | 94 |
| 선택된 박스 관측 RMS 중앙값 | 약 0.608px |
| 태그 자세 테스트 + episode CLI 테스트 | 12 + 5 통과 |

정확한 보정값·검토 영상 SHA256·검증 출처는
[box_tag_calibration.json](../pipeline/01_dataset/kuavo_standard/box_tag_calibration.json)에 있습니다.
랙 63개 부분집합, 박스 공동 8코너, 3태그 12코너, 이상 태그/코너 제외,
단일 추정 호환성과 회전된 오른쪽 태그의 단독 추정을 테스트했습니다.
기존 prepared world와 CURRENT의 로봇 결과는 보존했으며, 보정은 새 world 준비부터 적용됩니다.
