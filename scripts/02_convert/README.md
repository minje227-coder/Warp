# 녹화 변환

준비된 episode를 채택 WiLoR→KUAVO standard로 변환합니다.

```bash
bash scripts/02_convert/convert_episode_standard.sh /path/to/episode --dry-run
bash scripts/02_convert/convert_episode_standard.sh /path/to/episode
```

- `convert_episode_standard.sh` / `.py`: 해당 녹화의 prepared bundle 확인 후 standard 실행.
- `standard_kuavo.sh`: runtime의 `standard.py` 명령 실행.

입력 준비, runtime 선택과 4분할 출력은 [standard 안내](../../docs/RETARGETING_STANDARD.md)를 참고하세요.
LeRobot export·학습·RL은 이 episode 변환 명령 이후의 단계입니다.
