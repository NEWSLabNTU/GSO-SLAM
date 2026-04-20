# GSO-SLAM Repository Enumeration

> Reconstructed 2026-04-20 after prototype re-clone. Reflects the post-recovery state
> of the repo. Hardware / dependency versions verified against the live environment.

## Platform Profile

| Item | Value |
|------|-------|
| **Board** | NVIDIA Jetson AGX Orin Developer Kit |
| **L4T** | R36.4.4 (JetPack 6.1) |
| **CPU** | ARM Cortex-A78AE (aarch64) |
| **RAM** | 64 GB LPDDR5 |
| **GPU** | Ampere (sm_87), integrated |
| **CUDA Toolkit** | 12.6 |
| **cuDNN** | 9.3.0 |
| **TensorRT** | 10.3.0 |
| **OS** | Ubuntu 22.04 (L4T) |
| **Python** | 3.10.12 |
| **CMake** | 3.22.1 |
| **PyTorch** | 2.5.0a0 (NVIDIA JetPack wheel) |
| **numpy** | 1.26.1 (pinned < 2; see `memory/feedback_numpy_version.md`) |
| **Pangolin** | v0.8 (system) |
| **OpenCV** | 4.5.4 / 4.8.0 (no CUDA modules — see A.5 in RECOVERY.md) |

---

## Repository Structure (post-recovery)

```
GSO-SLAM/
├── src/
│   ├── main_dso_pangolin.cpp     # Entry point
│   ├── FullSystem/               # DSO tracking, mapping, optimisation
│   ├── GS/                       # Gaussian Splatting (model, trainer, renderer, mapper)
│   ├── OptimizationBackend/      # Energy functional, Hessian accumulation
│   ├── IOWrapper/                # Image I/O, Pangolin viewer
│   └── util/                     # Settings, calibration, dataset reader
├── include/                      # GS headers
├── cuda_rasterizer/              # Tile-based CUDA rasteriser (fwd/bwd)
├── viewer/                       # ImGui real-time viewer
├── thirdparty/
│   ├── Sophus/
│   ├── simple-knn/
│   ├── tinyply/
│   └── sse2neon/                 # DLTcollab fork @ 3b70b37 (upstream dead)
├── cfg/gaussian_mapper/Monocular/
│   ├── Replica/                  # 11 YAML configs
│   ├── TUM/                      # 4 YAML configs
│   └── Custom/                   # RECOVERY: my_scene.yaml (ZED XM)
├── dataset/                      # Download & preprocess scripts
├── experiments_bash/             # Run & eval scripts (replica/tum/MingDa)
│   └── scripts/                  # ATE / PSNR / SSIM / depth Python evals
├── tools/                        # RECOVERY: extract_rosbag.py, extract_rosbag_v2.py
├── docs/                         # RECOVERY: PLAN, ENUMERATION, ARCHITECTURE, RESULT, RECOVERY
├── justfile                      # RECOVERY: just recipes
├── cmake/
└── CMakeLists.txt
```

---

## Build Targets

| Target | Type | Purpose |
|--------|------|---------|
| `dso` | Static lib | DSO SLAM core |
| `gaussian_mapper` | Shared lib | Gaussian Splatting module |
| `cuda_rasterizer` | Shared lib | CUDA tile rasteriser |
| `simple_knn` | Shared lib | CUDA KNN |
| `imgui` | Shared lib | ImGui UI framework |
| `gaussian_viewer` | Shared lib | Real-time viewer |
| `dso_dataset` | Executable | Main SLAM binary (needs OpenCV + Pangolin) |

---

## Code-Level Notes (post Phase A)

- `CMakeLists.txt`: `cuda_rasterizer` and `simple_knn` set to `CUDA_ARCHITECTURES "87"` for Orin. `-DENABLE_SSE` gated behind x86 processor check. Eigen3::Eigen imported-target fallback is installed.
- `cuda_rasterizer/auxiliary.h`: `near_n = 0.01`, `p_view.z <= 0.01f` — aligned with YAML `Camera.z_near`.
- `src/GS/rasterize_points.cu`: uses `.data_ptr<T>()` (PyTorch 2.5 API).
- `src/GS/gaussian_model.cpp`: optimiser-state map keyed by `void*` (PyTorch 2.5 removed the string-keyed API).
- `src/GS/gaussian_mapper.cpp` + `include/{tensor_utils,camera,gaussian_mapper}.h`: all `cv::cuda::*` replaced with CPU equivalents. GPU transfer happens via torch tensors.
- `src/IOWrapper/Pangolin/*`: `<GL/glew.h>` included before Pangolin headers.
- `thirdparty/sse2neon/`: `DLTcollab/sse2neon@3b70b37`, with `SSE2NEON.h` symlink for case-sensitive includes.

---

## Known Incomplete Paths

| Location | Note |
|----------|------|
| `gaussian_mapper.cpp` STEREO branch | `stereo_cv_sgm_` never instantiated — unchanged from prototype |
| `gaussian_mapper.cpp:1604` | "support fisheye camera?" TODO |
| `main_dso_pangolin.cpp` | Some YAML keys not wired through; SLAM accepts them silently |

---

## Thread Architecture

| Thread | Entry Point | Purpose |
|--------|-------------|---------|
| Main/Tracking | `main_dso_pangolin.cpp` → `FullSystem::addActiveFrame()` | Pose estimation per frame |
| DSO Mapping | `FullSystem::mappingLoop()` | Keyframe creation, bundle adjustment |
| Gaussian Mapper | `GaussianMapper::run()` | GS training, depth feedback |
| ImGui Viewer | `ImGuiViewer::run()` | Real-time visualisation (optional) |

---

## Data Flow Summary

```
Image → CoarseTracker (pose) → Keyframe?
  → Yes: makeKeyFrame() → [WAIT for GS depth] → optimize() → freeze → GS queue
  → No:  makeNonKeyFrame() → trace & discard

GS queue → GaussianMapper::combineMappingOperations()
  → Create GaussianKeyframe → increasePcd() → trainForOneIteration()
  → renderKFDepths() → depth feedback → unblock DSO
```
