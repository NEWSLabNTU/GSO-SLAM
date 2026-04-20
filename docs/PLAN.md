# GSO-SLAM Build & Run Plan

> Reconstructed 2026-04-20 after prototype re-clone. `#UPDATE` annotations preserved
> from the 2026-03-16 build-up; `#UPDATE` entries dated 2026-04-20 reflect the
> post-reclone recovery work. See `RECOVERY.md` for the per-file change log.

## Goal

Build GSO-SLAM on Jetson AGX Orin, validate with a reference dataset, then run on
your own rosbag data to produce a Gaussian Splat map.

**End-to-end pipeline:**
```
Your ROS 2 bag → tools/extract_rosbag*.py → images/ + camera.txt + timestamps → dso_dataset → point_cloud.ply (GS map)
```

---

## Phase 1: Environment Setup (You)

These steps require sudo/manual intervention. Host state carried over the re-clone.

### 1.1 PyTorch for Jetson (CUDA 12.x)

Install the NVIDIA Jetson PyTorch wheel matching your JetPack (2.2.x/2.3.x/2.5.x for JP 6.x).

```bash
pip3 install numpy==1.26.1  # must stay < 2 (torch, OpenCV, ROS 2 all use numpy 1.x)
pip3 install <jetson-pytorch-wheel>.whl
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

> #UPDATE (2026-03-16): PyTorch 2.5.0a0 (JetPack 6.1 wheel) + numpy 1.26.1 installed.
>   Wheel: `torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl`.
>   Had to rename `/usr/lib/python3/dist-packages/torch` → `torch.bak` (import conflict).
>   Also installed `libcusparseLt`.
>
> #UPDATE (2026-04-20): Host still has torch 2.5.0a0 + numpy 1.26.1. No reinstall needed.

### 1.2 Pangolin

```bash
cd /tmp
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && git checkout v0.8
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && sudo make install && sudo ldconfig
```

> #UPDATE (2026-03-16): Pangolin v0.8 installed.
> #UPDATE (2026-04-20): Still installed system-wide — no action needed.

### 1.3 Git Submodules / sse2neon

```bash
cd ~/GSO-SLAM
git submodule update --init --recursive
# If the upstream submodule (jratcliff63367/sse2neon) is dead, clone the active fork:
rmdir thirdparty/sse2neon
git clone https://github.com/DLTcollab/sse2neon.git thirdparty/sse2neon
cd thirdparty/sse2neon && ln -sf sse2neon.h SSE2NEON.h
```

> #UPDATE (2026-03-16): Upstream dead; cloned `DLTcollab/sse2neon`. Added symlink.
> #UPDATE (2026-04-20): Re-applied after prototype re-clone. Commit `3b70b37`.

### 1.4 Rosbag data

Copy your ROS 2 bag(s) into `~/GSO-SLAM/data/`.

> #UPDATE (2026-03-16): `data/rosbag2_build_winding` + `data/rosbag2_2026_03_04_build_map`.
> #UPDATE (2026-04-20): Data directory not restored yet (user will handle separately).

---

## Phase 2: Code Fixes (Claude)

All fixes required to build on Jetson AGX Orin with JetPack 6.1 / CUDA 12.6 / PyTorch 2.5.
See `RECOVERY.md` for the complete per-file log.

### 2.1 CUDA architecture (`CMakeLists.txt`)

Set `cuda_rasterizer` and `simple_knn` to `CUDA_ARCHITECTURES "87"` for Orin.

> #UPDATE (2026-04-20): Done.

### 2.2 SSE/NEON gating (`CMakeLists.txt`)

Gate `-DENABLE_SSE` behind an x86/x86_64 `CMAKE_SYSTEM_PROCESSOR` check. ARM source
paths use the `sse2neon` translation layer via existing `#if !defined(__SSEn__)` guards.

> #UPDATE (2026-04-20): Done.

### 2.3 CUDA 12 / PyTorch 2.5 source compat

- `thirdparty/simple-knn/simple_knn.cu`: `#include <cfloat>` for `FLT_MAX`.
- `src/GS/rasterize_points.cu`: `.data<T>()` → `.data_ptr<T>()`.
- `src/GS/gaussian_model.cpp`: optimiser-state map keyed by `void*` (was
  `c10::guts::to_string`).
- `src/IOWrapper/Pangolin/{KeyFrameDisplay.cpp, PangolinDSOViewer.h}`: `#include <GL/glew.h>`
  before Pangolin headers.
- `CMakeLists.txt`: `Eigen3::Eigen` INTERFACE IMPORTED fallback for Pangolin.

> #UPDATE (2026-04-20): Done.

### 2.4 OpenCV CUDA → CPU fallback

JetPack 6.1 OpenCV ships without `cudawarping` / `cudaimgproc` / `cudastereo`.
Replaced:
- `cv::cuda::GpuMat` → `cv::Mat`
- `cv::cuda::resize` → `cv::resize`
- `cv::cuda::cvtColor` → `cv::cvtColor`
- `cv::cuda::reprojectImageTo3D` → `cv::reprojectImageTo3D`
- `cv::cuda::StereoSGM` → `cv::StereoSGBM`

Files: `include/{tensor_utils,camera,gaussian_mapper}.h`, `src/GS/gaussian_mapper.cpp`.

> #UPDATE (2026-04-20): Done. STEREO branch still lacks a `stereo_cv_sgm_` ctor
>   call; unchanged from prototype. Monocular works.

### 2.5 Near-plane constant (`cuda_rasterizer/auxiliary.h`)

Aligned `near_n` with YAML `Camera.z_near = 0.01`:
- `near_n = 0.2` → `0.01`
- `p_view.z <= 0.2f` → `0.01f`

> #UPDATE (2026-04-20): Done.

---

## Phase 3: Build (Claude)

### 3.1 Configure

```bash
cd ~/GSO-SLAM
mkdir -p build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DTorch_DIR=$(python3 -c "import torch; print(torch.utils.cmake_prefix_path)")/Torch
```

Or: `just configure`.

### 3.2 Compile

```bash
make -j$(nproc)   # or: just build
```

### 3.3 Verify

```bash
ls -la build/bin/dso_dataset
ldd build/bin/dso_dataset
```

> #UPDATE (2026-03-16): Build succeeded. Binary 250 MB, all `.so` dependencies resolved.
> #UPDATE (2026-04-20): Not re-run yet (deferred to Phase C after recovery).

---

## Phase 4: Validate with Reference Dataset

### 4.1 Download Replica

```bash
cd ~/GSO-SLAM/dataset
bash download_replica.sh
bash preprocess.sh
```

### 4.2 Run on Replica room0

```bash
cd ~/GSO-SLAM/experiments_bash
bash replica.sh   # runs all 8 scenes
# or: just replica-gui room0
```

Expected: ATE RMSE ~0.2 cm, ~10 fps, ~234 keyframes (see `RESULT.md`).

### 4.3 Verify Output

`experiments_bash/results/test/replica_room0/gs_map/point_cloud.ply` (~8 MB).

> #UPDATE (2026-03-16): ATE RMSE 0.20 cm on room0.

---

## Phase 5: Rosbag Extraction Pipeline

Two extractors are available under `tools/`:

| Script | Layout | When |
|--------|--------|------|
| `extract_rosbag.py` | Replica-style (`results/`, `camera.txt`, `pcalib.txt`, `vignette.png`, `times.txt`) | No groundtruth needed |
| `extract_rosbag_v2.py` | TUM-style (`rgb/`, `rgb.txt`, `camera.txt`, `pcalib.txt`, `vignette.png`, `groundtruth.txt`) | Odometry topic available — enables ATE eval |

Usage (TUM-format with odometry):
```bash
source /opt/ros/humble/setup.bash
python3 tools/extract_rosbag_v2.py \
  --bag data/rosbag2_build_winding \
  --image-topic /zedxm/zed_node/left/gray/rect/image \
  --camera-info-topic /zedxm/zed_node/left/gray/rect/camera_info \
  --odom-topic /visual_slam/tracking/odometry \
  --output data/my_scene_tum
```

**Key decisions:**
- Use rectified images (`image_rect_gray`) — distortion coefficients are zero.
- `bgr8`/`rgb8` images are converted to grayscale; `mono8` passes through.
- Timestamps extracted as ROS time (seconds).
- `pcalib.txt` must have all 256 values on a **single line** (space-separated).
- Stationary start of each bag must be trimmed via DSO's `start=` argument:
  - `rosbag2_build_winding` → start=800
  - `rosbag2_2026_03_04_build_map` → start=270

---

## Phase 6: Run on Custom Data

YAML config: `cfg/gaussian_mapper/Monocular/Custom/my_scene.yaml` (ZED XM: 960x600,
fx/fy=366.18, cx=464.15, cy=294.82, `Mapper.inactive_geo_densify=1`).

### 6.1 Run

```bash
# TUM-layout data with groundtruth
bash experiments_bash/MingDa.sh       # my_scene_tum, start=800, end=3400
bash experiments_bash/MingDa_5F.sh    # MingDa_5F_tum, start=270

# Or via justfile
just custom-gui-tum
just mingda5f-gui
```

### 6.2 Output

```
results/<scene>/
├── AllFramePose.txt
├── gs_map/
│   ├── point_cloud.ply      # Main output
│   ├── input.ply
│   ├── cameras.json
│   └── cfg_args
├── pose/
├── img/
├── map/
└── aff/
```

> #UPDATE (2026-03-16): 26 MB point_cloud.ply from rosbag2_build_winding run.
>   Critical: first ~800 stationary frames skipped via `start=800`.
>   `mode=1` required (no photometric calibration for real camera data).

---

## Phase 7: Optimisation for Orin

After basic functionality works, optimise for Jetson constraints:

- **Power / thermal**: `sudo nvpmodel -m 0 && sudo jetson_clocks`; monitor `tegrastats`.
- **Memory**: reduce `sh_degree`, resolution, sliding window size.
- **Profiling**: `nsys` / `ncu` for CUDA kernels; tune tile size in
  `cuda_rasterizer/config.h` for sm_87 occupancy.
- **Training cost**: tune iterations per keyframe, densification schedule.

---

## Risk Register

| Risk | Impact | Mitigation | Status |
|------|--------|------------|--------|
| No Jetson PyTorch wheel | High — cannot build | NVIDIA container / source build | Resolved — torch 2.5 wheel used |
| SSE intrinsics on ARM | Medium — compile errors | sse2neon translation | Resolved — gated + cloned |
| CUDA 12 API breaks | Low | Fix deprecated calls | Resolved — none found |
| Pangolin on aarch64 | Low | Well-tested; needs GLEW include | Resolved — v0.8 + GLEW include |
| GPU memory overflow (64 GB shared) | Medium | Reduce model size | Open |
| Rosbag encoding mismatch | Low | Script handles mono/bgr/rgb | Resolved |
| DSO tracking loss on custom data | Medium | Tune `mode=1`, trim stationary | Resolved |
| Scale ambiguity (monocular) | Medium | Use stereo if available | Open |
| Motion blur / fast motion | Medium | Subsample frames | Open |
| OpenCV CUDA modules missing | Medium | CPU fallback | Resolved |
| PyTorch 2.5 C++ API | Medium | `.data_ptr<T>()`, void* keys | Resolved |
| Old system torch cmake conflict | Medium | Remove stale Caffe2/Torch cmake | Resolved |
| Repo accidentally deleted + re-cloned | High | Recovery plan (`RECOVERY.md`) | Phase A done 2026-04-20; Phases B–D ongoing |

---

## Task Ownership

| Phase | Task | Owner | Status |
|-------|------|-------|--------|
| 1 | PyTorch install | You + Claude | Done |
| 1 | Pangolin install | You | Done |
| 1 | Submodules / sse2neon | Claude | Done (re-applied 2026-04-20) |
| 1 | Rosbag copy | You | Pending post-recovery |
| 2 | CUDA arch + SSE gate + CUDA 12 / torch 2.5 + CPU OpenCV + near plane | Claude | Done (re-applied 2026-04-20) |
| 3 | Build | Claude | Pending re-build (Phase C) |
| 4 | Replica validation | You + Claude | Pending re-run |
| 5 | Rosbag extraction scripts | Claude | Done (re-applied 2026-04-20) |
| 6 | Custom YAML + run | Claude | YAML done; run pending post-recovery |
| 7 | Performance tuning | Claude | Deferred |
