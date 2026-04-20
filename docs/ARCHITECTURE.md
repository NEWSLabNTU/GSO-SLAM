# GSO-SLAM System Architecture

> Reconstructed 2026-04-20 after prototype re-clone. Reflects the code structure as
> inherited from upstream with the post-recovery Phase A adjustments applied.

## 1. Pipeline / Dataflow

### Thread Architecture

The system runs **three concurrent threads**:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  MAIN / TRACKING THREAD                                                     │
│  (main_dso_pangolin.cpp)                                                    │
│                                                                             │
│  for each frame:                                                            │
│      FullSystem::addActiveFrame(image)                                      │
│          ├── trackNewCoarse()        ← fast pose estimation                 │
│          ├── decide keyframe?                                               │
│          └── deliverTrackedFrame()   → queue to mapping thread              │
└──────────────────────────────┬──────────────────────────────────────────────┘
                               │ unmappedTrackedFrames queue
                               ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  DSO MAPPING THREAD                                                         │
│  (FullSystem::mappingLoop)                                                  │
│                                                                             │
│  loop:                                                                      │
│      wait for queued frame                                                  │
│      ├── makeNonKeyFrame()  → trace points, discard frame                   │
│      └── makeKeyFrame()                                                     │
│            ├── traceNewCoarse()                                             │
│            ├── ■ BLOCK: wait for GS depth feedback ■                        │
│            ├── activatePoints()                                             │
│            ├── optimize() (bundle adjustment)                               │
│            ├── marginalize old frames                                       │
│            └── freeze KF → push to newframeHessians queue                   │
└──────────────────────────────┬──────────────────────────────────────────────┘
                               │ newframeHessians queue
                               │ (FrozenFrameHessian: pose, image,
                               │  sparse depth, 3D points, scales, rotations)
                               ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  GAUSSIAN MAPPER THREAD                                                     │
│  (GaussianMapper::run)                                                      │
│                                                                             │
│  Phase 1 — Initial Mapping:                                                 │
│      wait for ≥ min_num_initial_map_kfs from DSO                            │
│      consume all from newframeHessians                                      │
│      initialise GaussianModel from sparse points                            │
│      train initial model                                                    │
│                                                                             │
│  Phase 2 — Incremental Mapping:                                             │
│      loop:                                                                  │
│          if DSO requests update:                                            │
│              renderKFDepths() → dense depth from GS model                   │
│              ──── GS→DSO feedback (depth) ────▶ unblock DSO mapping         │
│          if new keyframes available:                                        │
│              combineMappingOperations()                                     │
│              add new Gaussians (increasePcd)                                │
│          trainForOneIteration()                                             │
│                                                                             │
│  Phase 3 — Final:                                                           │
│      render all keyframes, save PLY                                         │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Bidirectional Coupling

```
           DSO (Visual Odometry)                    Gaussian Splatting
          ┌─────────────────────┐                 ┌──────────────────────┐
          │ Keyframe poses      │ DSO → GS        │ GaussianKeyframe     │
          │ Sparse depth maps   │────────────────▶│ poses & images       │
          │ 3D points + colours │ (async, via     │                      │
          │ Scales & rotations  │ frozenMapMutex) │ GaussianModel        │
          │                     │                 │ (xyz, colour,        │
          │                     │ GS → DSO        │  opacity, scale,     │
          │ Point inverse-      │◀────────────────│  rotation tensors)   │
          │ depths updated      │ (blocking,      │                      │
          │ from rendered       │ per-keyframe)   │ Rendered dense       │
          │ dense depth         │                 │ depth maps           │
          └─────────────────────┘                 └──────────────────────┘

Synchronisation:
  DSO sets   callKFUpdateFromGS = true   ──▶ checked by GS each iteration
  DSO blocks on isDoneKFUpdateFromGS    ◀── set by GS after depth feedback
  DSO mapping thread spins with 1 ms sleep
```

---

## 2. Class Dependency Graph

### Top-level Ownership

```
main()
 │
 ├──▶ FullSystem ◁───────────────────────────────────────────┐
 │        │                                                   │
 │        │ (shared_ptr, bidirectional reference)             │
 │        │                                                   │
 ├──▶ GaussianMapper ──▶ pSLAM_ ──────────────────────────────┘
 │        │
 │        ├──▶ GaussianModel (shared_ptr)
 │        ├──▶ GaussianScene (shared_ptr)
 │        └──▶ Parameter objects (Model, Optimization, Pipeline)
 │
 └──▶ ImGuiViewer (optional, shared_ptr)
          ├──▶ ref to FullSystem
          └──▶ ref to GaussianMapper
```

### DSO Subsystem

```
FullSystem
 ├── owns ──▶ CoarseInitializer, CoarseTracker×2, PixelSelector, CoarseDistanceMap
 ├── owns ──▶ EnergyFunctional (EFFrame, EFPoint, EFResidual,
 │                              AccumulatedTopHessianSSE×2, AccumulatedSCHessianSSE,
 │                              IndexThreadReduce)
 ├── contains ─▶ CalibHessian Hcalib
 ├── manages ──▶ FrameHessian[] (frameHessians = active window)
 │                └── PointHessian[] / ImmaturePoint[] + PointFrameResidual[]
 ├── manages ──▶ FrameShell[] (allFrameHistory)
 └── manages ──▶ FrozenFrameHessian[] ← bridge to GaussianMapper
                  (camToWorld, kfImg, kfSparseDepth, pointsInWorld,
                   colors, scales, rots)
```

### Gaussian Splatting Subsystem

```
GaussianMapper
 ├── owns ──▶ GaussianModel
 │             ├── Tensors on GPU:
 │             │   xyz_, features_dc_, features_rest_, opacity_,
 │             │   scaling_, rotation_, xyz_gradient_accum_, denom_,
 │             │   max_radii2D_, exist_since_iter_
 │             ├── owns ──▶ torch::optim::Adam
 │             │              (state keyed by void* — see Phase A.3)
 │             └── Methods: createFromPcd, increasePcd,
 │                          densifyAndClone/Split, prunePoints,
 │                          trainingSetup, savePly, loadPly
 │
 ├── owns ──▶ GaussianScene
 │             ├── Camera[] (id, model, params, dist_coeff, undistort maps)
 │             └── GaussianKeyframe[] (Sophus::SE3d pose,
 │                                     image / depth / sparse_depth tensors,
 │                                     GPU transform tensors, Point2D[])
 │
 ├── uses ────▶ GaussianTrainer::trainingOnce(...)
 ├── uses ────▶ GaussianRenderer::render(...) → GaussianRasterizer
 │                                               └── torch::autograd::Function
 │                                                   ├── forward() → CUDA kernels
 │                                                   └── backward() → CUDA kernels
 │
 └── Parameter objects: GaussianModelParams, GaussianOptimizationParams,
                        GaussianPipelineParams
```

### CUDA Rasterisation Stack

```
GaussianRasterizer (C++ / PyTorch)
 ├── forward() → preprocessCUDA + renderCUDA (forward.cu, auxiliary.h)
 └── backward() → renderCUDA_backward (backward.cu)

Additional CUDA kernels (src/GS/):
 ├── rasterize_points.cu
 ├── operate_points.cu
 └── stereo_vision.cu
```

---

## 3. Synchronisation Map

```
FullSystem                          GaussianMapper
───────────                         ──────────────
trackMutex        protects CoarseTracker state
mapMutex          protects frameHessians, optimisation
                                    mutex_render_ protects model during render
frozenMapMutex    protects newframeHessians ◀──── also locked by GaussianMapper
                  and frameHessiansFrozen         when consuming frames
allKeyframeMutex  protects allKeyframeHessians
                                    mutex_settings_ protects param updates
shellPoseMutex    protects FrameShell poses
                                    GaussianScene::mutex_kfs_ protects KF map
                                    GaussianModel::mutex_settings_ protects LRs

Blocking sync flags:
  callKFUpdateFromGS (bool)  — checked by GS each iteration
  isDoneKFUpdateFromGS (bool) — set by GS after depth feedback
```

---

## 4. Key Boundary Structures

| Structure | Direction | Contents | Protected By |
|-----------|-----------|----------|-------------|
| `FrozenFrameHessian` | DSO → GS | pose, image, sparse depth, 3D points, colours, scales, rotations | `frozenMapMutex` |
| `newframeHessians` deque | DSO → GS | queue of FrozenFrameHessian for new KFs | `frozenMapMutex` |
| `frameHessiansFrozen` vector | DSO → GS | all frozen KFs (for pose/depth sync) | `frozenMapMutex` |
| Rendered depth (torch::Tensor) | GS → DSO | dense depth maps at KF viewpoints | `mapMutex` |
| `callKFUpdateFromGS` / `isDoneKFUpdateFromGS` | sync flags | trigger/ack GS→DSO update | atomic-like polling |
