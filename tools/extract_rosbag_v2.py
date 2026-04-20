#!/usr/bin/env python3
"""Extract a ROS 2 bag into a TUM RGB-D-style directory for GSO-SLAM.

The TUM layout is preferred when an odometry/groundtruth topic is available,
since `scripts/evaluate_ate_scale_tum.py` needs a `groundtruth.txt` to compute
ATE. Produces:

    <output>/
        rgb/                   # Sequential PNG frames (timestamp-named)
        rgb.txt                # "<timestamp> rgb/<filename>.png" per line
        camera.txt             # DSO pinhole calibration (normalised)
        pcalib.txt             # 256-value identity response curve (one line)
        vignette.png           # Uniform-white vignette image
        groundtruth.txt        # "<timestamp> tx ty tz qx qy qz qw" per line

Usage:
    source /opt/ros/humble/setup.bash
    python3 tools/extract_rosbag_v2.py \\
        --bag data/rosbag2_build_winding \\
        --image-topic /zedxm/zed_node/left/gray/rect/image \\
        --camera-info-topic /zedxm/zed_node/left/gray/rect/camera_info \\
        --odom-topic /visual_slam/tracking/odometry \\
        --output data/my_scene_tum
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import cv2
import numpy as np

# Re-use the DSO-format script's helpers for calibration/pcalib/vignette writing.
sys.path.insert(0, str(pathlib.Path(__file__).parent.resolve()))
from extract_rosbag import (  # noqa: E402
    _require_ros,
    write_camera_txt,
    write_pcalib_identity,
    write_uniform_vignette,
)


def extract_tum(bag_path: str, image_topic: str, camera_info_topic: str,
                odom_topic: str, output_dir: str,
                max_frames: int | None = None) -> None:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    out = pathlib.Path(output_dir)
    (out / "rgb").mkdir(parents=True, exist_ok=True)

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
    OdomMsg = get_message(type_map[odom_topic]) if odom_topic in type_map else None

    rgb_list = (out / "rgb.txt").open("w")
    rgb_list.write("# color images\n# timestamp filename\n")
    gt_list = (out / "groundtruth.txt").open("w")
    gt_list.write("# groundtruth trajectory\n# timestamp tx ty tz qx qy qz qw\n")

    intrinsics = None
    width = height = None
    frame_idx = 0

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        stamp_sec = stamp_ns * 1e-9

        if camera_info_topic and topic == camera_info_topic and intrinsics is None:
            info = deserialize_message(data, CameraInfoMsg)
            K = list(info.k)
            intrinsics = (K[0], K[4], K[2], K[5])
            width, height = info.width, info.height
            continue

        if odom_topic and topic == odom_topic and OdomMsg is not None:
            msg = deserialize_message(data, OdomMsg)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            gt_list.write(
                f"{stamp_sec:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
                f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n"
            )
            continue

        if topic != image_topic:
            continue

        msg = deserialize_message(data, ImageMsg)
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ("bgr8", "rgb8"):
            code = cv2.COLOR_BGR2GRAY if msg.encoding == "bgr8" else cv2.COLOR_RGB2GRAY
            arr = cv2.cvtColor(arr, code)
        else:
            arr = arr.reshape(msg.height, msg.width)

        fname = f"{stamp_sec:.6f}.png"
        cv2.imwrite(str(out / "rgb" / fname), arr)
        rgb_list.write(f"{stamp_sec:.6f} rgb/{fname}\n")
        frame_idx += 1

        if max_frames is not None and frame_idx >= max_frames:
            break

    rgb_list.close()
    gt_list.close()

    if intrinsics is None or width is None or height is None:
        sys.exit("no CameraInfo received — cannot write camera.txt")

    fx, fy, cx, cy = intrinsics
    write_camera_txt(out / "camera.txt", fx, fy, cx, cy, width, height)
    write_pcalib_identity(out / "pcalib.txt")
    write_uniform_vignette(out / "vignette.png", width, height)

    print(f"Extracted {frame_idx} frames to {out}")
    print(f"  camera: fx={fx:.3f} fy={fy:.3f} cx={cx:.3f} cy={cy:.3f} @ {width}x{height}")


def main() -> None:
    _require_ros()
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bag", required=True)
    p.add_argument("--image-topic", required=True)
    p.add_argument("--camera-info-topic", default="")
    p.add_argument("--odom-topic", default="",
                   help="Odometry topic for groundtruth.txt (e.g. "
                        "/visual_slam/tracking/odometry or /zedxm/zed_node/odom).")
    p.add_argument("--output", required=True)
    p.add_argument("--max-frames", type=int, default=None)
    args = p.parse_args()
    extract_tum(args.bag, args.image_topic, args.camera_info_topic,
                args.odom_topic, args.output, args.max_frames)


if __name__ == "__main__":
    main()
