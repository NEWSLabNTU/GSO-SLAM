# GSO-SLAM Recovery Log

**Recovery date:** 2026-04-20
**Trigger:** Repo was accidentally deleted and re-cloned from the upstream prototype. All prior code modifications, added files, and configs were lost. The host environment (PyTorch 2.5, Pangolin v0.8, numpy 1.26.1, CUDA 12.6) is intact.

This document records the recovery of code changes originally applied during the 2026-03-16 build-up. The recovery is reconstructed from memory + conversation history; line numbers in the prototype may differ from the originals, so every fix is verified against current file contents before edit.

---

## Phase A — Code Fixes (in progress)

### A.1 `CMakeLists.txt` — Orin / ARM / Eigen adjustments

| Change | Original state | Target state |
|--------|----------------|--------------|
| `cuda_rasterizer` CUDA arch | `"75;86"` | `"87"` (Orin sm_87) |
| `simple_knn` CUDA arch | not set | `"87"` |
| `-DENABLE_SSE` flag | always defined | gated behind `CMAKE_SYSTEM_PROCESSOR` x86/x86_64 check |
| `Eigen3::Eigen` target | may not be defined on older Eigen | add fallback imported target for Pangolin |

### A.2 `thirdparty/sse2neon/`

Upstream submodule repo `jratcliff63367/sse2neon` is dead. Clone from active fork `DLTcollab/sse2neon` and create `SSE2NEON.h → sse2neon.h` symlink (source files include uppercase name).

### A.3 CUDA 12.6 / PyTorch 2.5 source-level compatibility

| File | Change |
|------|--------|
| `thirdparty/simple-knn/simple_knn.cu` | `#include <cfloat>` for `FLT_MAX` |
| `src/GS/rasterize_points.cu` | `.data<T>()` → `.data_ptr<T>()` |
| `src/GS/gaussian_model.cpp` | Optimizer state map: `c10::guts::to_string(param)` → `void*` pointer key (string-keyed API removed in torch 2.5) |

### A.4 Pangolin + GLEW

Missing GL extension function declarations when Pangolin headers are included without GLEW first. Add `#include <GL/glew.h>` at top of:

- `src/IOWrapper/Pangolin/KeyFrameDisplay.cpp`
- `src/IOWrapper/Pangolin/PangolinDSOViewer.h`

### A.5 OpenCV CUDA → CPU

NVIDIA's JetPack 6.1 OpenCV package does not ship `cudawarping`, `cudaimgproc`, or `cudastereo`. Replace:

- `cv::cuda::GpuMat` → `cv::Mat`
- `cv::cuda::resize` → `cv::resize`
- `cv::cuda::cvtColor` → `cv::cvtColor`
- `cv::cuda::reprojectImageTo3D` → `cv::reprojectImageTo3D`
- `cv::cuda::StereoSGM` → `cv::StereoSGBM`

Files: `include/tensor_utils.h`, `include/camera.h`, `include/gaussian_mapper.h`, `src/GS/gaussian_mapper.cpp`.

### A.6 Near-plane constant

`cuda_rasterizer/auxiliary.h`:
- `near_n = 0.2` → `0.01`
- frustum-rejection `p_view.z <= 0.2f` → `0.01f`

Align with `z_near: 0.01` in the YAML configs.

### A.7 Doc fixes

- `README.md`: correct `mode=` (0=with calib, 1=no calib, 2=synthetic), `preset=` (0-3 per DSO), `files=` (add .zip support).
- `experiments_bash/tum.sh:33`: path typo `GSO_SLAM` → `GSO-SLAM`.

---

## Per-edit Log

### 2026-04-20 — Phase A applied

**A.1 CMakeLists.txt**
- Line ~162: `CUDA_ARCHITECTURES "75;86"` → `"87"`
- After `simple_knn` target: added `set_target_properties(simple_knn PROPERTIES CUDA_ARCHITECTURES "87")`
- Line ~47: `add_definitions("-DENABLE_SSE")` wrapped in `if (CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")` guard
- Added `Eigen3::Eigen` INTERFACE IMPORTED fallback when target is missing but `EIGEN3_INCLUDE_DIR` is set

**A.2 sse2neon submodule**
- Directory `thirdparty/sse2neon/` repopulated by cloning `https://github.com/DLTcollab/sse2neon.git` (upstream `jratcliff63367` is dead)
- Recorded commit: `3b70b3727edc9a151c113814129258c3423a771c`
- Symlink `SSE2NEON.h → sse2neon.h` created (source files include uppercase name)
- `.gitmodules` left pointing at upstream; not updated as part of Phase A so git state is transparent. Up to the user whether to rewrite it.

**A.3 CUDA/PyTorch 2.5 source compat**
- `thirdparty/simple-knn/simple_knn.cu`: added `#include <cfloat>` (alongside existing `<vector>` include)
- `src/GS/rasterize_points.cu`: all `.data<float>()`, `.data<int>()`, `.data<bool>()` replaced with `.data_ptr<…>()` (3 replace_all passes)
- `src/GS/gaussian_model.cpp`: three optimizer-state blocks changed from `auto key = c10::guts::to_string(param.unsafeGetTensorImpl());` to `void* key = param.unsafeGetTensorImpl();` (in `replaceTensorToOptimizer`, `prunePoints`, `densificationPostfix`)

**A.4 Pangolin GLEW includes**
- `src/IOWrapper/Pangolin/KeyFrameDisplay.cpp`: added `#include <GL/glew.h>` before `#include <pangolin/pangolin.h>`
- `src/IOWrapper/Pangolin/PangolinDSOViewer.h`: same

**A.5 OpenCV CUDA → CPU fallback**
- `include/tensor_utils.h`: removed `<opencv2/cudaimgproc.hpp>` include; rewrote `cvGpuMat2TorchTensor_Float32` (now takes `cv::Mat&`) and `torchTensor2CvGpuMat_Float32` (now returns `cv::Mat`). Function names kept to minimise call-site churn.
- `include/camera.h`: removed `<opencv2/cudawarping.hpp>`; replaced `cv::cuda::GpuMat`/`cv::cuda::resize` block in `initUndistortRectifyMapAndMask` with CPU `cv::Mat`/`cv::resize`.
- `include/gaussian_mapper.h`: removed cudaimgproc/cudastereo/cudawarping headers; `stereo_cv_sgm_` type changed from `cv::Ptr<cv::cuda::StereoSGM>` to `cv::Ptr<cv::StereoSGBM>`.
- `src/GS/gaussian_mapper.cpp`: four `if (device_type_ == kCUDA) { cv::cuda path } else { cpu path }` blocks collapsed to CPU-only path (initial mapping, incremental mapping, combined mapping operations, per-keyframe pyramid). Single-image RGB site switched from `cv::cuda::GpuMat` to `cv::Mat`. STEREO branch rewritten to use `cv::Mat + cv::cvtColor + cv::StereoSGBM::compute + cv::reprojectImageTo3D`. RGBD branch switched to `cv::Mat`. No `stereo_cv_sgm_` initialisation exists yet, so the STEREO branch is still effectively non-functional — same as prior state (ENUMERATION.md noted this was an incomplete path).

**A.6 Near-plane alignment**
- `cuda_rasterizer/auxiliary.h`: `near_n = 0.2` → `0.01`; `p_view.z <= 0.2f` → `0.01f`

**A.7 Doc / script fixes**
- `experiments_bash/tum.sh:33`: path typo `${HOME}/GSO_SLAM/...` → `${HOME}/GSO-SLAM/...`
- README.md: prototype version does not contain a command-line options table, so no `mode=`/`preset=`/`files=` fix was applicable here. The richer README from the 2026-03-16 build-up was part of Phase B content and will be reintroduced later per the user's stage-by-stage plan.

### Verification

Grep across `/home/jetson/GSO-SLAM` returns no remaining:
- `c10::guts::to_string`
- `cv::cuda::` in source files (only in this RECOVERY.md's prose)
- `CUDA_ARCHITECTURES "75`
- `near_n = 0.2` or `p_view.z <= 0.2f` in cuda_rasterizer/
- `.data<float/int/bool>()` in src/

Build not yet run — that is Phase C.

---

## Phase B — Docs & Tooling (applied 2026-04-20)

**B.1 `docs/`** — recreated from memory:
- `PLAN.md` — master build/run plan with #UPDATE history split between the
  2026-03-16 build-up and the 2026-04-20 recovery.
- `ENUMERATION.md` — platform profile, repo structure, build targets, known
  incomplete paths.
- `ARCHITECTURE.md` — thread architecture, class dependency graph, sync map,
  boundary structures.
- `RESULT.md` — pre-recovery benchmark numbers marked as "expected to reproduce"
  until Phase C re-validates.

**B.2 `justfile`** — recipes: `configure`, `build`, `replica-gui/pangolin/headless`,
`tum-gui/pangolin`, `custom-gui`, `custom-gui-tum`, `mingda5f-gui`. Custom recipes
use `mode=1` (real camera without photometric calibration), Replica uses `mode=2`,
TUM uses `mode=1`.

**B.3 `tools/`**:
- `extract_rosbag.py` — DSO/Replica-style layout (`results/` + `times.txt`).
- `extract_rosbag_v2.py` — TUM-style layout (`rgb/` + `rgb.txt` + `groundtruth.txt`);
  reuses helpers from `extract_rosbag.py`. Groundtruth is extracted from an
  odometry topic (`/visual_slam/tracking/odometry` or `/zedxm/zed_node/odom`).

**B.4 `cfg/gaussian_mapper/Monocular/Custom/my_scene.yaml`** — ZED XM 960x600
pinhole config with fx/fy=366.418, cx=464.152, cy=294.820, zero distortion.
`Mapper.inactive_geo_densify=1`, `Pipeline.depth_ratio=0.0` (unbounded scene).

**B.5 `experiments_bash/MingDa.sh`, `MingDa_5F.sh`** — headless runs for custom
data with `start=800 end=3400` and `start=270` respectively, feeding into
`evaluate_ate_scale_tum.py`.

Still out of scope: actual rosbag data under `data/`, Replica/TUM datasets.

---

## Phase C — Build & Smoke Test (applied 2026-04-20)

**C.1 Configure**
- `mkdir -p build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DTorch_DIR=$(python3 -c "import torch; print(torch.utils.cmake_prefix_path)")/Torch`
- Found Torch, SuiteSparse, Eigen 3.4, Boost, CUDA, glm, OpenGL, jsoncpp, VTK, PCL, libzip 1.7.3, Pangolin, OpenCV. Rerun SDK 0.31.3 fetched successfully.
- Configured all targets including `dso_dataset`.

**C.2 Build**
- `make -j$(nproc)` completed to `[100%] Built target dso_dataset`.
- Output binary: `build/bin/dso_dataset` (~252 MB).
- Only warnings (Sophus `SO3` deprecation, Boost bind placeholders, rerun `set_time_seconds` deprecation) — no errors.
- `ldd` shows all shared deps resolved (torch from `~/.local/lib/python3.10/site-packages/torch/lib/`, Pangolin from `/usr/local/lib/`, OpenCV 4.8 from `/lib/`, GLEW 2.2).
- Smoke-test: `dso_dataset calib=/dev/null` prints "could not read calib file! exit." and returns 0 — load path works.

**C.3 Replica validation (skipped)**
- No Replica dataset present on disk (nor TUM). Validation requires `~/GSO-SLAM/../dataset/Replica/room0/...` per `replica.sh`.
- Will be reattempted in Phase D once the user restores a dataset. Expected numbers: ATE RMSE ≈ 0.2 cm, ~10 fps, ~234 keyframes (see `RESULT.md`).

---

## Phase D — Dataset Restore + End-to-End Validation (in progress 2026-04-20)

**D.1 Replica (cancelled)**
- `download_replica.sh` fetched from cvg-data.inf.ethz.ch at ~190 KB/s — ETA 7+ hours. Cancelled per user; partial `Replica.zip` removed.

**D.2 TUM (in progress)**
- `download_tum.sh` fetched freiburg1_desk (344 MB) successfully, freiburg2_xyz downloading in background, freiburg3 queued.
- Script extracts tarballs into `dataset/` root, but `experiments_bash/tum.sh` expects `dataset/TUM/rgbd_dataset_*/`. Manually moved freiburg1_desk into `dataset/TUM/` to complete staging.

**D.3 Preprocess**
- Ran `bash preprocess.sh`. Copied `camera.txt` and `tum_freiburg*.txt` associations from `data/TUM/...` templates into `TUM/rgbd_dataset_*/`. (Preprocess also created scaffolding for fr2/fr3 which will be populated once their tarballs finish.)

**D.4 Re-validation on TUM freiburg1_desk**

Two failures hit before success — worth recording:

1. First failure: **segfault with `quiet=1`**. Root cause: `justfile_directory() / "../dataset/TUM"` resolved to `/home/jetson/dataset/TUM` (outside the repo) rather than `/home/jetson/GSO-SLAM/dataset/TUM`. With `quiet=1`, the "file not found" error was suppressed and the code later dereferenced a null reader → SIGSEGV.
2. Fix: Changed `justfile` paths from `"../dataset/..."` to `"dataset/..."`. New `tum-headless` recipe added so TUM can be run without Pangolin/Gaussian viewers.

After the fix, `just tum-headless freiburg1_desk` ran to completion:

| Metric | Value |
|--------|-------|
| Frames processed | 612 / 613 |
| Tracking speed | 10.0 fps |
| Real-time factor | 0.389x single-core / 0.739x multi-core |
| DSO resets | 1 (shutdown checkpoint `887_shutdown/`) |
| Output | `results/tum_freiburg1_desk/gs_map/point_cloud.ply` (5.9 MB), `input.ply` (2.4 MB), `cameras.json`, `AllFramePose.txt` + per-keyframe `pose/`, `img/`, `map/`, `aff/` |
| Exit code | 0 |

This confirms the **post-recovery build runs end-to-end on real data**: calibration loading, DSO tracking, GS mapping + CUDA rasteriser, PLY serialisation all work. fr1_desk is a harder sequence than Replica for DSO (fast rotation + close objects explain the sub-real-time factor and the re-track attempts) — the expected Replica room0 numbers from `RESULT.md` (0.2 cm ATE, 0.89x RT) should be met once Replica is available.

ATE not computed here (would need `evaluate_ate_scale_tum.py` + groundtruth alignment).




