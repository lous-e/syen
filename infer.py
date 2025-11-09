# # stream_predict.py / stream_predict_knn.py
# # ------------------------
# # Real-time inference helpers for streaming IMU rows over serial.
# #
# # This file contains two parts:
# #  - commented-out SVM stream example (kept for reference)
# #  - active kNN-based real-time inference that loads `knn_digit_cls.joblib`
# #
# # Usage:
# #  - Update PORT and BAUD to match your serial device
# #  - Run this script while the ESP32 is streaming rows; it will print
# #    predictions for completed writing segments.

# PORT="COM9"
# BAUD=115200
# pipe = load("svm_digit_cls.joblib")

# def parse_row(line):
#     # t_ms,ax,ay,az,gx,gy,gz,state,label
#     parts = line.strip().split(",")
#     if len(parts)!=9: return None
#     try:
#         rec = {
#           "t_ms": int(parts[0]),
#           "ax": float(parts[1]), "ay": float(parts[2]), "az": float(parts[3]),
#           "gx": float(parts[4]), "gy": float(parts[5]), "gz": float(parts[6]),
#           "state": int(parts[7]),
#           "label": parts[8],   # unused at inference
#         }
#         return rec
#     except: return None

# def df_to_pred(df):
#     dfw = keep_writing_segment(df)
#     if dfw.empty: return None
#     x = trial_features(dfw).reshape(1, -1)
#     return int(pipe.predict(x)[0])

# def main():
#     ser = serial.Serial(PORT, BAUD, timeout=1)
#     # optional reset dance:
#     ser.setDTR(False); ser.setRTS(True); time.sleep(0.1)
#     ser.setDTR(True);  ser.setRTS(False); time.sleep(0.5)

#     buf = []
#     in_write = False
#     print("Listening...")
#     while True:
#         line = ser.readline().decode("utf-8", errors="ignore")
#         if not line: continue
#         rec = parse_row(line)
#         if not rec: continue

#         st = rec["state"]
#         if st == 1:
#             if not in_write:
#                 # new segment
#                 buf = [rec]; in_write = True
#             else:
#                 buf.append(rec)
#         else:  # st == 0
#             if in_write:
#                 # segment ended — run prediction
#                 in_write = False
#                 df = pd.DataFrame(buf)
#                 pred = df_to_pred(df)
#                 if pred is not None:
#                     print(f"[PREDICT] digit = {pred}")
#                 else:
#                     print("[SKIP] too short/noisy segment")
#             # else: idle

# if __name__ == "__main__":
#     main()

# stream_predict_knn.py
# Real-time inference over serial using the kNN feature pipeline

import serial, time, numpy as np, pandas as pd
from joblib import load

MODEL = load("knn_digit_cls.joblib")
PIPE  = MODEL["pipe"]
CHANNELS = MODEL["channels"]

PORT="COM9"; BAUD=115200

def keep_writing_segment(df: pd.DataFrame) -> pd.DataFrame:
    df = df[df["state"] == 1].copy()
    if len(df) < 8: return pd.DataFrame()
    trim_ms = 200
    if "t_ms" in df.columns and df["t_ms"].is_monotonic_increasing:
        t0, t1 = df["t_ms"].iloc[0], df["t_ms"].iloc[-1]
        left, right = t0 + trim_ms, t1 - trim_ms
        df = df[(df["t_ms"] >= left) & (df["t_ms"] <= right)]
        if len(df) < 8: df = df.iloc[2:-2] if len(df) > 4 else df
    else:
        k = max(2, len(df)//20)
        if len(df) > 2*k: df = df.iloc[k:-k]
    return df

def trial_features(df: pd.DataFrame) -> np.ndarray:
    X = df[CHANNELS].to_numpy().astype(float)
    mu, sd = X.mean(axis=0), X.std(axis=0) + 1e-6
    Xn = (X - mu) / sd
    ax, ay, az, gx, gy, gz = [Xn[:,i] for i in range(6)]

    dur_s = 0.0
    if "t_ms" in df.columns and df["t_ms"].is_monotonic_increasing:
        dur_s = float((df["t_ms"].iloc[-1] - df["t_ms"].iloc[0]) / 1000.0)

    a_path = float(np.sum(np.linalg.norm(Xn[:, :3], axis=1)))
    g_path = float(np.sum(np.linalg.norm(Xn[:, 3:], axis=1)))
    yaw_int = float(np.sum(np.abs(gz)))

    def feat_channel(x):
        if x.size == 0: return [0]*8
        mean = float(np.mean(x))
        std  = float(np.std(x))
        rms  = float(np.sqrt(np.mean(x*x)))
        rng  = float(np.max(x) - np.min(x))
        mad  = float(np.median(np.abs(x - np.median(x))))
        eng  = float(np.sum(x*x))
        zc   = int(np.sum((x[:-1]*x[1:]) < 0))
        # cheap peak count without scipy at inference:
        thr = np.std(x)*1.0
        peaks = int(np.sum((np.abs(x[1:-1])>thr) &
                           (np.abs(x[1:-1])>=np.abs(x[:-2])) &
                           (np.abs(x[1:-1])>=np.abs(x[2:]))))
        return [mean, std, rms, rng, mad, eng, zc, peaks]

    feats = []
    for ch in [ax, ay, az, gx, gy, gz]:
        feats += feat_channel(ch)
    feats += [dur_s, a_path, g_path, yaw_int]
    return np.array(feats, dtype=float)

def parse_row(line):
    # t_ms,ax,ay,az,gx,gy,gz,state,label
    p = line.strip().split(",")
    if len(p)!=9: return None
    try:
        return {
          "t_ms": int(p[0]),
          "ax": float(p[1]), "ay": float(p[2]), "az": float(p[3]),
          "gx": float(p[4]), "gy": float(p[5]), "gz": float(p[6]),
          "state": int(p[7]),
          "label": p[8],
        }
    except: return None

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    # optional reset:
    ser.setDTR(False); ser.setRTS(True); time.sleep(0.1)
    ser.setDTR(True);  ser.setRTS(False); time.sleep(0.5)

    buf, in_write = [], False
    print("Listening...")
    while True:
        line = ser.readline().decode("utf-8", errors="ignore")
        if not line: continue
        rec = parse_row(line)
        if not rec: continue

        if rec["state"] == 1:
            if not in_write:
                buf = [rec]; in_write = True
            else:
                buf.append(rec)
        else:
            if in_write:
                in_write = False
                df = pd.DataFrame(buf)
                dfw = keep_writing_segment(df)
                if dfw.empty:
                    print("[SKIP] too short/noisy segment")
                    continue
                x = trial_features(dfw).reshape(1, -1)
                pred = int(PIPE.predict(x)[0])
                print(f"[PREDICT] digit = {pred}")

if __name__ == "__main__":
    main()