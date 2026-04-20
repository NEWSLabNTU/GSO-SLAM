#!/usr/bin/env python3
"""Extract a ROS 2 bag into a DSO/Replica-style directory for GSO-SLAM.

Produces the directory layout expected by the Replica-style loader:

    <output>/
        results/            # Sequential PNG frames (00000.png, 00001.png, ...)
        camera.txt          # DSO pinhole calibration (normalised)
        pcalib.txt          # 256-value identity response curve (one line)
        vignette.png        # Uniform-white vignette image
        times.txt           # "<id> <timestamp_sec> 0.0" per line

Source-on-disk requirements: ROS 2 Humble (`rclpy`, `rosbag2_py`,
`sensor_msgs`, `cv_bridge`). Run with the ROS environment sourced.

Usage:
    source /opt/ros/humble/setup.bash
    python3 tools/extract_rosbag.py \\
        --bag data/rosbag2_build_winding \\
        --image-topic /zedxm/zed_node/left/gray/rect/image \\
        --camera-info-topic /zedxm/zed_node/left/gray/rect/camera_info \\
        --output data/my_scene
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import cv2
import numpy as np


def _require_ros():
    try:
        import rclpy  # noqa: F401
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions  # noqa: F401
        from rclpy.serialization import deserialize_message  # noqa: F401
        from rosidl_runtime_py.utilities import get_message  # noqa: F401
    except Exception as exc:
        sys.exit(
            "ROS 2 Python packages not importable — did you run "
            "`source /opt/ros/humble/setup.bash`? (" + str(exc) + ")"
        )


def write_camera_txt(path: pathlib.Path, fx: float, fy: float, cx: float, cy: float,
                     width: int, height: int) -> None:
    """Write a DSO pinhole calibration with normalised (fx/w, fy/h) parameters."""
    nfx, nfy = fx / width, fy / height
    ncx, ncy = cx / width, cy / height
    with path.open("w") as f:
        # Pinhole parameters relative to image dimensions + zero distortion
        f.write(f"{nfx:.8f} {nfy:.8f} {ncx:.8f} {ncy:.8f} 0\n")
        f.write(f"{width} {height}\n")
        # Target rectified intrinsics (same, since input is already rectified)
        f.write(f"{nfx:.8f} {nfy:.8f} {ncx:.8f} {ncy:.8f} 0\n")
        f.write(f"{width} {height}\n")


def write_pcalib_identity(path: pathlib.Path) -> None:
    """Identity gamma / photometric response: 256 linearly-spaced values on one line."""
    values = " ".join(f"{v:.3f}" for v in np.linspace(0.0, 255.0, 256))
    path.write_text(values + "\n")


def write_uniform_vignette(path: pathlib.Path, width: int, height: int) -> None:
    """Uniform-white 16-bit vignette (no attenuation)."""
    vign = np.full((height, width), 65535, dtype=np.uint16)
    cv2.imwrite(str(path), vign)


def extract(bag_path: str, image_topic: str, camera_info_topic: str,
            output_dir: str, max_frames: int | None = None) -> None:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    out = pathlib.Path(output_dir)
    (out / "results").mkdir(parents=True, exist_ok=True)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr",
                         output_serialization_format="cdr"),
    )

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if image_topic not in type_map:
        sys.exit(f"image topic '{image_topic}' not present in bag")
    ImageMsg = get_message(type_map[image_topic])
    CameraInfoMsg = (get_message(type_map[camera_info_topic])
                     if camera_info_topic in type_map else None)

    times_f = (out / "times.txt").open("w")
    intrinsics = None
    width = height = None
    frame_idx = 0
    first_ts = None

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if camera_info_topic and topic == camera_info_topic and intrinsics is None:
            info = deserialize_message(data, CameraInfoMsg)
            K = list(info.k)
            intrinsics = (K[0], K[4], K[2], K[5])  # fx, fy, cx, cy
            width, height = info.width, info.height
            continue
        if topic != image_topic:
            continue

        msg = deserialize_message(data, ImageMsg)
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        # GSO-SLAM wants mono8 internally. Convert colour → gray when needed.
        if msg.encoding in ("bgr8", "rgb8"):
            code = cv2.COLOR_BGR2GRAY if msg.encoding == "bgr8" else cv2.COLOR_RGB2GRAY
            arr = cv2.cvtColor(arr, code)
        else:
            arr = arr.reshape(msg.height, msg.width)

        stamp_sec = stamp_ns * 1e-9
        if first_ts is None:
            first_ts = stamp_sec
        rel = stamp_sec - first_ts

        cv2.imwrite(str(out / "results" / f"{frame_idx:05d}.png"), arr)
        times_f.write(f"{frame_idx} {rel:.9f} 0.0\n")
        frame_idx += 1

        if max_frames is not None and frame_idx >= max_frames:
            break

    times_f.close()

    if intrinsics is None or width is None:
        sys.exit("no CameraInfo received — cannot write camera.txt")
    if height is None:
        sys.exit("CameraInfo did not carry an image size")

    fx, fy, cx, cy = intrinsics
    write_camera_txt(out / "camera.txt", fx, fy, cx, cy, width, height)
    write_pcalib_identity(out / "pcalib.txt")
    write_uniform_vignette(out / "vignette.png", width, height)

    print(f"Extracted {frame_idx} frames to {out}")
    print(f"  camera: fx={fx:.3f} fy={fy:.3f} cx={cx:.3f} cy={cy:.3f} @ {width}x{height}")


def main() -> None:
    _require_ros()
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bag", required=True, help="Path to rosbag2 directory (.db3 sibling metadata.yaml).")
    p.add_argument("--image-topic", required=True)
    p.add_argument("--camera-info-topic", default="")
    p.add_argument("--output", required=True)
    p.add_argument("--max-frames", type=int, default=None)
    args = p.parse_args()
    extract(args.bag, args.image_topic, args.camera_info_topic, args.output, args.max_frames)


if __name__ == "__main__":
    main()
