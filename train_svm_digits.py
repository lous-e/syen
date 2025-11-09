# train_knn_digits.py
# ===================
# Training script for KNN-based digit classifier.
#
# This script scans a folder containing CSV logs of IMU "writing" gestures
# (files named like `digit_<label>_<timestamp>.csv`), extracts per-trial
# features and trains a K-Nearest-Neighbors model. The trained model is
# saved to `knn_digit_cls.joblib` by default.
#
# Requirements:
#  pip install scikit-learn pandas numpy joblib scipy

import re, numpy as np, pandas as pd
from pathlib import Path
from scipy.signal import find_peaks
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import StratifiedKFold, cross_val_score
from sklearn.metrics import classification_report, confusion_matrix
from joblib import dump
import argparse

DATA_DIR = Path("4_seconds")         # folder with digit_*.csv
MODEL_OUT = Path("knn_digit_cls.joblib")
digit_pat = re.compile(r"digit_([0-9])_")
CHANNELS = ["ax","ay","az","gx","gy","gz"]

# ---------- utils ----------
def label_from_name(p: Path) -> int:
    m = digit_pat.search(p.name)
    if not m: raise ValueError(f"Cannot parse label from filename: {p}")
    return int(m.group(1))

def keep_writing_segment(df: pd.DataFrame) -> pd.DataFrame:
    """Keep only rows with state==1 and trim ~200 ms at both ends."""
    if "state" not in df.columns:
        raise ValueError("CSV must include a 'state' column.")
    df = df[df["state"] == 1].copy()
    if len(df) < 8: return pd.DataFrame()

    trim_ms = 200
    if "t_ms" in df.columns and df["t_ms"].is_monotonic_increasing:
        t0, t1 = df["t_ms"].iloc[0], df["t_ms"].iloc[-1]
        left, right = t0 + trim_ms, t1 - trim_ms
        df = df[(df["t_ms"] >= left) & (df["t_ms"] <= right)]
        if len(df) < 8:
            df = df.iloc[2:-2] if len(df) > 4 else df
    else:
        k = max(2, len(df)//20)
        if len(df) > 2*k: df = df.iloc[k:-k]
    return df

def feat_channel(x: np.ndarray):
    """Per-channel stats: mean,std,RMS,range,MAD,energy,zero-cross,peak-count."""
    if x.size == 0: return [0]*8
    x = x.astype(float)
    mean = float(np.mean(x))
    std  = float(np.std(x))
    rms  = float(np.sqrt(np.mean(x*x)))
    rng  = float(np.max(x) - np.min(x))
    mad  = float(np.median(np.abs(x - np.median(x))))
    eng  = float(np.sum(x*x))
    zc   = int(np.sum((x[:-1]*x[1:]) < 0))
    peaks, _ = find_peaks(np.abs(x), height=np.std(x)*1.0)
    return [mean, std, rms, rng, mad, eng, zc, int(len(peaks))]

def trial_features(df: pd.DataFrame) -> np.ndarray:
    """Aggregate features per trial (no leakage from 'state' or 'label')."""
    X = df[CHANNELS].to_numpy().astype(float)

    # per-trial z-score (helps invariance to pose/scale)
    mu, sd = X.mean(axis=0), X.std(axis=0) + 1e-6
    Xn = (X - mu) / sd
    ax, ay, az, gx, gy, gz = [Xn[:,i] for i in range(6)]

    dur_s = 0.0
    if "t_ms" in df.columns and df["t_ms"].is_monotonic_increasing:
        dur_s = float((df["t_ms"].iloc[-1] - df["t_ms"].iloc[0]) / 1000.0)

    # global kinematics
    a_path = float(np.sum(np.linalg.norm(Xn[:, :3], axis=1)))
    g_path = float(np.sum(np.linalg.norm(Xn[:, 3:], axis=1)))
    yaw_int = float(np.sum(np.abs(gz)))

    feats = []
    for ch in [ax, ay, az, gx, gy, gz]:
        feats += feat_channel(ch)
    feats += [dur_s, a_path, g_path, yaw_int]
    return np.array(feats, dtype=float)

def load_dataset(data_dir: Path):
    X, y, files = [], [], []
    for fp in sorted(data_dir.glob("digit_*.csv")):
        try:
            df = pd.read_csv(fp)
        except Exception:
            continue
        dfw = keep_writing_segment(df)
        if dfw.empty: continue
        feats = trial_features(dfw)
        X.append(feats); y.append(label_from_name(fp)); files.append(str(fp))
    if not X:
        raise RuntimeError("No usable samples found. Check folder/CSVs.")
    return np.vstack(X), np.array(y), files

def main():
    k = 3
    weights = "distance"
    metric = "euclidean"
    # ap = argparse.ArgumentParser()
    # ap.add_argument("--k", type=int, default=3, help="k in kNN (try 1,2,5)")
    # ap.add_argument("--weights", type=str, default="distance",
    #                 choices=["uniform","distance"], help="vote weighting")
    # ap.add_argument("--metric", type=str, default="",
    #                 choices=["euclidean","manhattan","minkowski"], help="distance metric")
    # args = ap.parse_args()

    X, y, files = load_dataset(DATA_DIR)
    print(f"Loaded {len(y)} samples. Classes: {sorted(set(y))}. Features: {X.shape[1]}")

    pipe = Pipeline([
        ("sc", StandardScaler()),
        ("knn", KNeighborsClassifier(n_neighbors=k,
                                     metric=metric,
                                     weights=weights))
    ])

    cv = StratifiedKFold(n_splits=5, shuffle=True, random_state=0)
    scores = cross_val_score(pipe, X, y, cv=cv, scoring="accuracy")
    print(f"kNN (k={k}, {weights}, {metric}) 5-fold accuracy: "
          f"{scores.mean():.3f} +/- {scores.std():.3f}")

    pipe.fit(X, y)
    dump({"pipe": pipe, "channels": CHANNELS}, MODEL_OUT)
    print(f"[saved] {MODEL_OUT}")

    yhat = pipe.predict(X)
    print("\nIn-sample report (sanity only):")
    print(classification_report(y, yhat, digits=3))
    print("Confusion matrix:\n", confusion_matrix(y, yhat))

if __name__ == "__main__":
    main()