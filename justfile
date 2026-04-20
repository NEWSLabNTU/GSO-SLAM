# GSO-SLAM justfile
default:
    @echo "Usage: just <recipe> [args]"
    @echo
    @just --list

# Paths
build_dir    := justfile_directory() / "build"
bin          := build_dir / "bin" / "dso_dataset"
cfg_dir      := justfile_directory() / "cfg/gaussian_mapper/Monocular"
replica_dir  := justfile_directory() / "dataset/Replica"
tum_dir      := justfile_directory() / "dataset/TUM"

# Build the project
build jobs="$(nproc)":
    cmake --build {{build_dir}} -j{{jobs}}

# Configure the project (first-time cmake run)
configure:
    mkdir -p {{build_dir}}
    cmake -S {{justfile_directory()}} -B {{build_dir}} \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
        -DTorch_DIR=$(python3 -c "import torch; print(torch.utils.cmake_prefix_path)")/Torch

# ─── Replica dataset (with GUI) ──────────────────────────────────────────────

# Run Replica scene with both Pangolin 3D viewer and Gaussian viewer
# Example: just replica-gui room0
replica-gui scene="room0":
    {{bin}} \
        files="{{replica_dir}}/{{scene}}/results" \
        calib="{{replica_dir}}/{{scene}}/camera.txt" \
        gamma="{{replica_dir}}/{{scene}}/pcalib.txt" \
        vignette="{{replica_dir}}/{{scene}}/vignette.png" \
        preset=0 mode=2 \
        which_dataset=replica \
        cfg_yaml="{{cfg_dir}}/Replica/{{scene}}.yaml" \
        save_dir="results/{{scene}}" \
        use_gaussian_viewer=1

# Run Replica scene with Pangolin 3D viewer only (no Gaussian viewer)
replica-pangolin scene="room0":
    {{bin}} \
        files="{{replica_dir}}/{{scene}}/results" \
        calib="{{replica_dir}}/{{scene}}/camera.txt" \
        gamma="{{replica_dir}}/{{scene}}/pcalib.txt" \
        vignette="{{replica_dir}}/{{scene}}/vignette.png" \
        preset=0 mode=2 \
        which_dataset=replica \
        cfg_yaml="{{cfg_dir}}/Replica/{{scene}}.yaml" \
        save_dir="results/{{scene}}" \
        use_gaussian_viewer=0

# ─── TUM dataset (with GUI) ─────────────────────────────────────────────────

# Run TUM scene with both viewers
# Example: just tum-gui freiburg1_desk
tum-gui scene="freiburg1_desk":
    {{bin}} \
        files="{{tum_dir}}/rgbd_dataset_{{scene}}/rgb" \
        calib="{{tum_dir}}/rgbd_dataset_{{scene}}/camera.txt" \
        gamma="{{tum_dir}}/rgbd_dataset_{{scene}}/pcalib.txt" \
        vignette="{{tum_dir}}/rgbd_dataset_{{scene}}/vignette.png" \
        dataassociation="{{tum_dir}}/rgbd_dataset_{{scene}}/rgb.txt" \
        preset=0 mode=1 \
        cfg_yaml="{{cfg_dir}}/TUM/tum_{{scene}}.yaml" \
        save_dir="results/tum_{{scene}}" \
        use_gaussian_viewer=1

# Run TUM scene with Pangolin 3D viewer only
tum-pangolin scene="freiburg1_desk":
    {{bin}} \
        files="{{tum_dir}}/rgbd_dataset_{{scene}}/rgb" \
        calib="{{tum_dir}}/rgbd_dataset_{{scene}}/camera.txt" \
        gamma="{{tum_dir}}/rgbd_dataset_{{scene}}/pcalib.txt" \
        vignette="{{tum_dir}}/rgbd_dataset_{{scene}}/vignette.png" \
        dataassociation="{{tum_dir}}/rgbd_dataset_{{scene}}/rgb.txt" \
        preset=0 mode=1 \
        cfg_yaml="{{cfg_dir}}/TUM/tum_{{scene}}.yaml" \
        save_dir="results/tum_{{scene}}" \
        use_gaussian_viewer=0

# ─── Custom dataset (with GUI) ──────────────────────────────────────────────

# Run custom scene (DSO-style directory) with both viewers
# Example: just custom-gui data/my_scene_trimmed/results data/my_scene_trimmed/camera.txt
custom-gui images_dir=("data/my_scene_trimmed/results") calib_file=("data/my_scene_trimmed/camera.txt") cfg_yaml=(cfg_dir / "Custom/my_scene.yaml"):
    {{bin}} \
        files="{{images_dir}}" \
        calib="{{calib_file}}" \
        gamma=data/my_scene_trimmed/pcalib.txt \
        vignette=data/my_scene_trimmed/vignette.png \
        cfg_yaml="{{cfg_yaml}}" \
        save_dir="results/custom_deleteME" \
        which_dataset=replica \
        preset=0 mode=1 \
        use_gaussian_viewer=1

# Run custom scene with TUM-style layout (has groundtruth for ATE eval).
# Example: just custom-gui-tum
custom-gui-tum start="800" end="3400":
    {{bin}} \
        files="data/my_scene_tum/rgb" \
        calib="data/my_scene_tum/camera.txt" \
        gamma="data/my_scene_tum/pcalib.txt" \
        vignette="data/my_scene_tum/vignette.png" \
        dataassociation="data/my_scene_tum/rgb.txt" \
        preset=0 mode=1 \
        cfg_yaml="{{cfg_dir}}/Custom/my_scene.yaml" \
        save_dir="results/my_scene_tum" \
        start={{start}} end={{end}} \
        use_gaussian_viewer=1

# Run MingDa_5F TUM-format custom scene (start=270 skips ~13.6s stationary)
mingda5f-gui start="270":
    {{bin}} \
        files="data/MingDa_5F_tum/rgb" \
        calib="data/MingDa_5F_tum/camera.txt" \
        gamma="data/MingDa_5F_tum/pcalib.txt" \
        vignette="data/MingDa_5F_tum/vignette.png" \
        dataassociation="data/MingDa_5F_tum/rgb.txt" \
        preset=0 mode=1 \
        cfg_yaml="{{cfg_dir}}/Custom/my_scene.yaml" \
        save_dir="results/MingDa_5F" \
        start={{start}} \
        use_gaussian_viewer=1

# ─── Headless (no GUI) ──────────────────────────────────────────────────────

# Run Replica scene headless (no GUI, quiet)
replica-headless scene="room0":
    {{bin}} \
        files="{{replica_dir}}/{{scene}}/results" \
        calib="{{replica_dir}}/{{scene}}/camera.txt" \
        gamma="{{replica_dir}}/{{scene}}/pcalib.txt" \
        vignette="{{replica_dir}}/{{scene}}/vignette.png" \
        preset=0 mode=2 quiet=1 nogui=1 \
        which_dataset=replica \
        cfg_yaml="{{cfg_dir}}/Replica/{{scene}}.yaml" \
        save_dir="results/{{scene}}" \
        use_gaussian_viewer=0

# Run TUM scene headless (no GUI, quiet)
tum-headless scene="freiburg1_desk":
    {{bin}} \
        files="{{tum_dir}}/rgbd_dataset_{{scene}}/rgb" \
        calib="{{tum_dir}}/rgbd_dataset_{{scene}}/camera.txt" \
        gamma="{{tum_dir}}/rgbd_dataset_{{scene}}/pcalib.txt" \
        vignette="{{tum_dir}}/rgbd_dataset_{{scene}}/vignette.png" \
        dataassociation="{{tum_dir}}/rgbd_dataset_{{scene}}/rgb.txt" \
        preset=0 mode=1 quiet=1 nogui=1 \
        cfg_yaml="{{cfg_dir}}/TUM/tum_{{scene}}.yaml" \
        save_dir="results/tum_{{scene}}" \
        use_gaussian_viewer=0
