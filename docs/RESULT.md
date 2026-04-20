# GSO-SLAM Results on Jetson AGX Orin

> Reconstructed 2026-04-20 after prototype re-clone. Results below are from the
> 2026-03-16/26 build-up on the same hardware. They describe what the post-recovery
> system is **expected to reproduce** — they have not been re-measured since the
> re-clone. Phase C will re-validate at least Replica room0.

## Platform

| Spec | Value |
|------|-------|
| Device | NVIDIA Jetson AGX Orin 64 GB |
| GPU | Ampere (sm_87), shared memory with CPU |
| JetPack | 6.1 (L4T R36) |
| CUDA | 12.6 |
| PyTorch | 2.5.0a0 (NVIDIA JetPack wheel) |
| OpenCV | 4.5.4 / 4.8.0 (CPU-only, no CUDA modules) |

---

## 1. Replica room0 (Reference Benchmark)

Synthetic indoor dataset, 1200x680, 2000 frames.

### Tracking

| Metric | Value |
|--------|-------|
| ATE RMSE | **0.20 cm** |
| Frames processed | 2000 |
| Keyframes | 234 |
| Tracking speed | 10.0 fps |
| Real-time factor | 0.89x (multi-core) |
| Tracking resets | 2 (frame 824, frame 1194) |

### Gaussian Splatting Rendering Quality

Metrics are per-keyframe, evaluated at each tracking-reset checkpoint.

| Metric | Checkpoint @824 | Checkpoint @1194 |
|--------|----------------|-----------------|
| PSNR mean | 20.47 dB | **25.76 dB** |
| PSNR median | 21.01 dB | 26.08 dB |
| PSNR min | 11.91 dB | 14.50 dB |
| PSNR max | 24.17 dB | 32.10 dB |
| SSIM mean | 0.271 | 0.182 |
| Render time mean | 69.1 ms | 57.3 ms |
| Render FPS | 14.5 | 17.5 |

### Gaussian Splat Map

| Property | Value |
|----------|-------|
| File | `experiments_bash/results/test/replica_room0/gs_map/point_cloud.ply` |
| Size | 8.1 MB |
| Gaussians | 126,612 |
| SH degree | 0 (DC only, grayscale) |

---

## 2. Custom Data: ZED XM Building Winding Sequence

Real-world indoor/outdoor sequence captured with ZED XM stereo camera (left eye
monocular), 960x600, ~29.3 fps, ~222 s.

### Data Preparation Notes

- First ~800 frames (27 s) are stationary and must be skipped (DSO cannot initialise
  without camera motion).
- `mode=1` (no photometric calibration).
- Rectified grayscale images, zero distortion.
- Camera: fx=fy=366.18, cx=464.15, cy=294.82.

### Tracking

| Metric | Value |
|--------|-------|
| Frames processed | 5725 (trimmed from 6526) |
| Keyframes | 1026 |
| Tracking speed | 10.0 fps |
| Real-time factor | 1.03x (multi-core) |
| Tracking resets | 1 (frame 4326) |

### Gaussian Splatting Rendering Quality

Metrics evaluated at tracking-reset checkpoint (frame 4326), across 1033 keyframes.

| Metric | Value |
|--------|-------|
| PSNR mean | **18.35 dB** |
| PSNR median | 18.79 dB |
| PSNR min | 8.27 dB |
| PSNR max | 24.06 dB |
| SSIM mean | 0.191 |
| Render time mean | 37.7 ms |
| Render FPS | **26.5** |

### Gaussian Splat Map

| Property | Value |
|----------|-------|
| File | `results/my_scene/gs_map/point_cloud.ply` |
| Size | 26.4 MB |
| Gaussians | 412,510 |
| SH degree | 0 (DC only, grayscale) |

---

## Comparison Summary

| Metric | Replica room0 | Custom ZED XM |
|--------|--------------|---------------|
| PSNR (dB) | **25.76** | 18.35 |
| SSIM | 0.182 | 0.191 |
| Render FPS | 17.5 | **26.5** |
| Gaussians | 126K | 412K |
| Map size | 8.1 MB | 26.4 MB |
| Tracking speed | 10 fps | 10 fps |
| Real-time factor | 0.89x | **1.03x** |

The lower PSNR on custom data is expected: Replica is a noise-free synthetic
dataset, while the ZED XM data has sensor noise, motion blur, and varying
illumination. The rendering speed is higher on custom data due to the smaller
image resolution (960x600 vs 1200x680).

---

## How to View the Gaussian Splat Map

Output at `results/<scene>/gs_map/point_cloud.ply` (positions, normals, SH colour
coefficients, opacity, scale, rotation).

**Gaussian Splatting Viewers (recommended):**
- [SuperSplat](https://playcanvas.com/supersplat/editor) — web, drag-and-drop `.ply`
- [splaTV](https://github.com/antimatter15/splat) — lightweight web viewer
- [GaussianSplats3D](https://github.com/mkkellogg/GaussianSplats3D) — Three.js
- SIBR Viewer (original 3DGS paper) — desktop app, supports fly-through

**General 3D viewers (points only, no splat rendering):**
- MeshLab, CloudCompare, Open3D
