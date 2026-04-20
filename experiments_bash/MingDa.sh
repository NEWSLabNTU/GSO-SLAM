#!/bin/bash
# Run GSO-SLAM on the ZED XM "my_scene" rosbag (TUM-format extraction).
# Start offset 800 skips the ~27 s of stationary frames at the beginning of
# rosbag2_build_winding. End offset 3400 trims to the meaningful segment.
set -e

DATA_DIR="${HOME}/GSO-SLAM/data/my_scene_tum"
RESULT_DIR="./results/test"
SAVE_PATH="${RESULT_DIR}/my_scene_tum"
OUTPUT_TXT="${RESULT_DIR}/my_scene_results.txt"

mkdir -p "${SAVE_PATH}"

echo "--- MingDa (my_scene_tum) Evaluation Start ---" > "${OUTPUT_TXT}"
echo "========================================"
echo "Processing: my_scene_tum"

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
    start=800 \
    end=3400 \
    cfg_yaml="${HOME}/GSO-SLAM/cfg/gaussian_mapper/Monocular/Custom/my_scene.yaml" \
    save_dir="${SAVE_PATH}" \
    use_gaussian_viewer=0

echo "Evaluating ATE for my_scene_tum..."
python3 -W ignore scripts/evaluate_ate_scale_tum.py \
    "${DATA_DIR}" \
    "${SAVE_PATH}" \
    "${DATA_DIR}/rgb.txt" >> "${OUTPUT_TXT}"

echo "Done: my_scene_tum"
echo "Results saved in ${OUTPUT_TXT}"
