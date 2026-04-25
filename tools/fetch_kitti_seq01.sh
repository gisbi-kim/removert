#!/usr/bin/env bash
# Fetch KITTI odometry sequence 01 inputs without cvlibs.net registration.
#
# Sources:
#   - AWS S3 mirror (avg-kitti) for raw 2011_10_03_drive_0042 (= odom seq 01)
#     and the corresponding calib bundle.
#   - semantic-kitti.org for SuMa poses + semantic labels (drives sequence 01
#     evaluation and gives us the same pose source the Removert paper used).
#
# Layout produced under data/:
#   data/raw/
#     2011_10_03_calib.zip
#     2011_10_03_drive_0042_sync.zip
#   data/semantickitti/
#     data_odometry_labels.zip
#
# Reorganization into KITTI-odometry layout (sequences/01/velodyne/*.bin +
# poses.txt + calib.txt) is intentionally NOT done here — that is the job of
# the P0 KITTI -> HDF5 converter (tools/kitti_to_h5.py, to be written).

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DATA="$ROOT/data"
RAW="$DATA/raw"
SK="$DATA/semantickitti"

mkdir -p "$RAW" "$SK"

fetch() {
    local url="$1" dest="$2"
    if [[ -s "$dest" ]]; then
        echo "[skip] $dest already exists"
        return
    fi
    echo "[get ] $url"
    # -C - resumes partial downloads; --retry survives flaky links.
    curl -fL --retry 5 --retry-delay 2 -C - -o "$dest" "$url"
}

fetch "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_calib.zip" \
      "$RAW/2011_10_03_calib.zip"

fetch "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_10_03_drive_0042/2011_10_03_drive_0042_sync.zip" \
      "$RAW/2011_10_03_drive_0042_sync.zip"

fetch "http://semantic-kitti.org/assets/data_odometry_labels.zip" \
      "$SK/data_odometry_labels.zip"

echo
echo "Done. Files under $DATA:"
ls -lh "$RAW" "$SK"
