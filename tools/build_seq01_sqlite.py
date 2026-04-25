#!/usr/bin/env python3
"""Build a SQLite DB for KITTI odometry sequence 01 (= raw 2011_10_03_drive_0042).

Inputs (already downloaded under data/ by tools/fetch_kitti_seq01.sh):
  data/raw/2011_10_03_calib.zip                - calib_{cam_to_cam,velo_to_cam,imu_to_velo}.txt
  data/raw/2011_10_03_drive_0042_sync.zip      - velodyne, 4 cameras, oxts, timestamps
  data/semantickitti/data_odometry_labels.zip  - SuMa-estimated poses for seq 01

Output (default): data/seq01.sqlite

Schema:
  meta(key, value)                          - provenance + conventions
  calib(key, rows, cols, data BLOB)         - all KITTI calib matrices, float64
  frames(frame_idx, timestamp_ns, pose)     - pose = 4x4 float32 BLOB, row-major
  scans(frame_idx, num_points, points)      - points = (N, 4) float32: x, y, z, intensity
  images(frame_idx, cam_id, w, h, fmt, data)- PNG bytes per (frame, camera)

Pose convention (matches KITTI odometry GT / SemanticKITTI SuMa):
  pose[i] transforms a point in cam0 coords at frame i to cam0 coords at frame 0.
  To get world_from_lidar at frame i: pose[i] @ Tr (Tr = velo_to_cam_R augmented to 4x4).
  Tr is stored under calib keys 'velo_to_cam_R' and 'velo_to_cam_T'.
"""

from __future__ import annotations

import argparse
import sqlite3
import time
import zipfile
from datetime import datetime
from pathlib import Path

import numpy as np


SCHEMA = """
CREATE TABLE meta   (key TEXT PRIMARY KEY, value TEXT NOT NULL);
CREATE TABLE calib  (key TEXT PRIMARY KEY, rows INTEGER NOT NULL,
                     cols INTEGER NOT NULL, data BLOB NOT NULL);
CREATE TABLE frames (frame_idx INTEGER PRIMARY KEY, timestamp_ns INTEGER NOT NULL,
                     pose_cam0 BLOB NOT NULL);
CREATE TABLE scans  (frame_idx INTEGER PRIMARY KEY,
                     num_points INTEGER NOT NULL, points BLOB NOT NULL,
                     FOREIGN KEY (frame_idx) REFERENCES frames(frame_idx));
CREATE TABLE images (frame_idx INTEGER NOT NULL, cam_id INTEGER NOT NULL,
                     width INTEGER NOT NULL, height INTEGER NOT NULL,
                     format TEXT NOT NULL, data BLOB NOT NULL,
                     PRIMARY KEY (frame_idx, cam_id),
                     FOREIGN KEY (frame_idx) REFERENCES frames(frame_idx));
CREATE INDEX idx_images_cam ON images(cam_id);
"""


# -- parsers ----------------------------------------------------------------

def parse_kv_floats(text: str) -> dict[str, np.ndarray]:
    """KITTI calib format: '<key>: <floats>' per line. Skip non-numeric values."""
    out: dict[str, np.ndarray] = {}
    for line in text.splitlines():
        if ":" not in line:
            continue
        key, rest = line.split(":", 1)
        key = key.strip()
        if key in {"calib_time", "corner_dist"}:
            continue
        try:
            out[key] = np.array([float(x) for x in rest.split()], dtype=np.float64)
        except ValueError:
            continue
    return out


def calib_shape(key: str, n: int) -> tuple[int, int]:
    if key.startswith(("P_rect", "P0", "P1", "P2", "P3")) and n == 12:
        return 3, 4
    if key.startswith(("R_rect", "R_")) and n == 9:
        return 3, 3
    if key in {"R"} and n == 9:
        return 3, 3
    if key in {"T"} and n == 3:
        return 3, 1
    if key.startswith(("K_",)) and n == 9:
        return 3, 3
    if key.startswith(("D_",)):
        return 1, n
    if key.startswith(("S_", "S")) and n == 2:
        return 1, 2
    if n == 16:
        return 4, 4
    if n == 12:
        return 3, 4
    return 1, n


def parse_timestamps(text: str) -> list[int]:
    """KITTI timestamps.txt -> int nanoseconds since Unix epoch."""
    out = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        date_part, time_part = line.split(" ", 1)
        hms, frac = (time_part.split(".", 1) + ["0"])[:2]
        dt = datetime.strptime(f"{date_part} {hms}", "%Y-%m-%d %H:%M:%S")
        epoch_s = int(dt.timestamp())
        frac_ns = int((frac + "0" * 9)[:9])
        out.append(epoch_s * 1_000_000_000 + frac_ns)
    return out


def parse_poses(text: str) -> np.ndarray:
    """KITTI poses.txt -> (N, 4, 4) float32. Each line = 12 floats (3x4 row-major)."""
    rows = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        vals = np.fromstring(line, sep=" ", dtype=np.float32)
        if vals.size != 12:
            raise ValueError(f"pose line has {vals.size} floats, expected 12")
        T = np.eye(4, dtype=np.float32)
        T[:3, :4] = vals.reshape(3, 4)
        rows.append(T)
    return np.stack(rows, axis=0) if rows else np.zeros((0, 4, 4), np.float32)


def png_dims(b: bytes) -> tuple[int, int]:
    """Read width/height from PNG IHDR chunk (offsets 16..23)."""
    return int.from_bytes(b[16:20], "big"), int.from_bytes(b[20:24], "big")


# -- builders ---------------------------------------------------------------

def insert_meta(db: sqlite3.Connection, cams: list[int]) -> None:
    db.executemany(
        "INSERT INTO meta(key, value) VALUES(?, ?)",
        [
            ("source_drive",    "2011_10_03_drive_0042_sync"),
            ("odometry_seq",    "01"),
            ("pose_source",     "SemanticKITTI labels.zip:dataset/sequences/01/poses.txt (SuMa)"),
            ("pose_semantics",  "pose[i] transforms cam0 frame i -> cam0 frame 0; cam0 = world"),
            ("pose_layout",     "4x4 float32 row-major"),
            ("point_dtype",     "float32"),
            ("point_layout",    "x,y,z,intensity (4 floats per point)"),
            ("calib_dtype",     "float64"),
            ("included_cams",   ",".join(str(c) for c in cams) if cams else ""),
            ("built_at_unix_s", str(int(time.time()))),
        ],
    )


def insert_calib(db: sqlite3.Connection, calib_zip: Path) -> None:
    with zipfile.ZipFile(calib_zip) as z:
        cam_text  = z.read("2011_10_03/calib_cam_to_cam.txt").decode()
        velo_text = z.read("2011_10_03/calib_velo_to_cam.txt").decode()
        imu_text  = z.read("2011_10_03/calib_imu_to_velo.txt").decode()

    rows: list[tuple] = []
    for k, v in parse_kv_floats(cam_text).items():
        r, c = calib_shape(k, v.size)
        if r * c != v.size:
            r, c = 1, v.size
        rows.append((k, r, c, v.tobytes()))
    for src, prefix in ((velo_text, "velo_to_cam_"), (imu_text, "imu_to_velo_")):
        for k, v in parse_kv_floats(src).items():
            r, c = calib_shape(k, v.size)
            if r * c != v.size:
                r, c = 1, v.size
            rows.append((prefix + k, r, c, v.tobytes()))
    db.executemany("INSERT INTO calib(key, rows, cols, data) VALUES(?, ?, ?, ?)", rows)


def load_poses(labels_zip: Path) -> np.ndarray:
    with zipfile.ZipFile(labels_zip) as z:
        text = z.read("dataset/sequences/01/poses.txt").decode()
    return parse_poses(text)


def insert_frames(db: sqlite3.Connection, drive_zip: Path, poses: np.ndarray,
                  cams: list[int], limit: int) -> int:
    base = "2011_10_03/2011_10_03_drive_0042_sync/"
    with zipfile.ZipFile(drive_zip) as zd:
        names = set(zd.namelist())
        velo_files = sorted(n for n in names
                            if n.startswith(base + "velodyne_points/data/")
                            and n.endswith(".bin"))
        timestamps = parse_timestamps(
            zd.read(base + "velodyne_points/timestamps.txt").decode())

        n_frames = len(velo_files)
        if limit > 0:
            n_frames = min(n_frames, limit)
        n_use = min(n_frames, len(poses), len(timestamps))
        if n_use < n_frames:
            print(f"  WARN: truncating to {n_use} (poses={len(poses)}, "
                  f"ts={len(timestamps)}, scans={n_frames})")

        cur = db.cursor()
        cur.execute("BEGIN")
        try:
            for i in range(n_use):
                cur.execute(
                    "INSERT INTO frames(frame_idx, timestamp_ns, pose_cam0) "
                    "VALUES(?, ?, ?)",
                    (i, timestamps[i], poses[i].tobytes()))

                scan = zd.read(base + f"velodyne_points/data/{i:010d}.bin")
                cur.execute(
                    "INSERT INTO scans(frame_idx, num_points, points) "
                    "VALUES(?, ?, ?)",
                    (i, len(scan) // 16, scan))

                for cam in cams:
                    img_path = base + f"image_0{cam}/data/{i:010d}.png"
                    if img_path not in names:
                        continue
                    img = zd.read(img_path)
                    w, h = png_dims(img)
                    cur.execute(
                        "INSERT INTO images(frame_idx, cam_id, width, height, "
                        "format, data) VALUES(?, ?, ?, ?, 'png', ?)",
                        (i, cam, w, h, img))

                if (i + 1) % 100 == 0 or (i + 1) == n_use:
                    print(f"  frame {i + 1}/{n_use}")
            cur.execute("COMMIT")
        except Exception:
            cur.execute("ROLLBACK")
            raise
    return n_use


# -- entry ------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root",  default=".",                 help="Repo root")
    ap.add_argument("--out",   default="data/seq01.sqlite", help="Output DB path")
    ap.add_argument("--cams",  default="2",
                    help="Comma-separated cam ids to include (0,1,2,3); '' = none")
    ap.add_argument("--limit", type=int, default=0,
                    help="Limit frames for quick smoke test (0 = all)")
    args = ap.parse_args()

    root = Path(args.root).resolve()
    out  = root / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        out.unlink()

    cams = [int(c) for c in args.cams.split(",") if c.strip()] if args.cams else []

    print(f"[init ] db = {out}")
    # isolation_level=None disables Python's implicit transactions so we can
    # use explicit BEGIN/COMMIT (one big tx for the bulk insert is far faster).
    db = sqlite3.connect(str(out), isolation_level=None)
    db.execute("PRAGMA journal_mode=WAL")
    db.execute("PRAGMA synchronous=NORMAL")
    db.executescript(SCHEMA)

    print("[meta ] inserting meta")
    insert_meta(db, cams)

    print("[calib] reading + inserting calib")
    insert_calib(db, root / "data/raw/2011_10_03_calib.zip")

    print("[pose ] loading SuMa poses")
    poses = load_poses(root / "data/semantickitti/data_odometry_labels.zip")
    print(f"[pose ] got {len(poses)} poses")

    print(f"[frame] inserting frames + scans + cams={cams}")
    n = insert_frames(db,
                      root / "data/raw/2011_10_03_drive_0042_sync.zip",
                      poses, cams, args.limit)

    print("[opt  ] PRAGMA optimize")
    db.execute("PRAGMA optimize")
    db.commit()
    db.close()

    mb = out.stat().st_size / (1024 * 1024)
    print(f"\nDone: {out}  ({n} frames, {mb:.1f} MB)")


if __name__ == "__main__":
    main()
