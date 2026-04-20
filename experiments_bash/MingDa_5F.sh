#!/bin/bash
# Run GSO-SLAM on MingDa_5F (rosbag2_2026_03_04_build_map, TUM-format extraction).
# Start offset 270 skips the ~13.6 s of stationary frames at the beginning;
# DSO initialises at approximately frame 273.
set -e

DATA_DIR="${HOME}/GSO-SLAM/data/MingDa_5F_tum"
RESULT_DIR="./results/test"
SAVE_PATH="${RESULT_DIR}/MingDa_5F"
OUTPUT_TXT="${RESULT_DIR}/MingDa_5F_results.txt"

mkdir -p "${SAVE_PATH}"

echo "--- MingDa_5F Evaluation Start ---" > "${OUTPUT_TXT}"
echo "========================================"
echo "Processing: MingDa_5F"

../build/bin/dso_dataset \
    files="${DATA_DIR}/rgb" \
    calib="${DATA_DIR}/camera.txt" \
    gamma="${DATA_DIR}/pcalib.txt" \
    vignette="${DATA_DIR}/vignette.png" \
    dataassociation="${DATA_DIR}/rgb.txt" \
    preset=0 \
    mode=1 \
    quiet=1 \
    nogui=1 \
    start=270 \
    cfg_yaml="${HOME}/GSO-SLAM/cfg/gaussian_mapper/Monocular/Custom/my_scene.yaml" \
    save_dir="${SAVE_PATH}" \
    use_gaussian_viewer=0

echo "Evaluating ATE for MingDa_5F..."
python3 -W ignore scripts/evaluate_ate_scale_tum.py \
    "${DATA_DIR}" \
    "${SAVE_PATH}" \
    "${DATA_DIR}/rgb.txt" >> "${OUTPUT_TXT}"

echo "Done: MingDa_5F"
echo "Results saved in ${OUTPUT_TXT}"
