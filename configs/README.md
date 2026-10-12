# Configuration

Host, Quest, planned capture streams and selectable modules are set with `bash scripts/setup.sh`.

- `configs/local.env` is generated locally and **gitignored**, so machine-specific choices do not get committed.
- Runtime-specific robot geometry, camera calibration, TCP, and rack models should be added here later as documented, non-sensitive configurations.
- Selecting a planned data stream does **not** implement the RGB+Skeleton synchronized recorder.

See [`docs/SETUP_OPTIONS.md`](../docs/SETUP_OPTIONS.md).
