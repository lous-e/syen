# syen — IMU digit collection and digit classifiers

This repository contains firmware and Python utilities for collecting "digit" writing gestures from a 6-DoF IMU (LSM6DSOX) attached to an ESP32, extracting features, training simple classifiers (kNN/SVM) and running real-time inference over serial.

Contents
- `ml_digits.ino` — ESP32 logger that streams CSV rows for every IMU sample.
- `write.py` — Python helper to collect a single labeled writing segment and save it to `imu_digits/`.
- `train_svm_digits.py` / `train_knn_digits.py` — training scripts (feature extraction + model training).
- `infer.py` — serial realtime inference using a trained `knn_digit_cls.joblib` model.
- `collect` folders (`imu_digits/`, `4_seconds/`, etc.) hold recorded CSVs.
- `morse.ino`, `firmware.ino` — other firmware examples in the repo.

Quickstart
1. Install Python deps:

   pip install numpy pandas scikit-learn joblib scipy pyserial

2. Collect a sample (on Windows):
   - Set `PORT` in `write.py` (e.g. `COM3`) and run `python write.py`.
   - Type a digit label (0–9), then write the digit on the device; the script will save a CSV to `imu_digits/`.

3. Train a classifier:
   - Edit the training script to point at your data directory (look for `DATA_DIR`).
   - Run the script (e.g. `python train_svm_digits.py` or the KNN script) to produce a `.joblib` model file.

4. Real-time inference:
   - Ensure the model (`knn_digit_cls.joblib`) is present in the repo root.
   - Edit `PORT` in `infer.py` and run `python infer.py` to stream from the board and see predictions.

Security & secrets
- `morse.ino` contains placeholders for Wi‑Fi and Telegram tokens. Do NOT commit real secrets into the repo. Instead use a local, untracked header or build-time configuration to inject them.

Notes and suggestions
- CSV format: `t_ms,ax,ay,az,gx,gy,gz,state,label` — `state` marks writing vs rest.
- If your CSVs contain formatting issues (e.g. `-` characters as placeholders), use `pandas.to_numeric(..., errors='coerce')` and `.fillna()` before numeric conversion.
- This repository is intentionally lightweight and designed for experimentation. Consider adding unit tests for feature extraction and a small example dataset to make training reproducible.