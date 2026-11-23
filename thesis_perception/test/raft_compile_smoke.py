#!/usr/bin/env python3
"""Compare TorchVision RAFT eager and compiled FP32 flows on recorded frames."""
import os
from pathlib import Path
import sys

import cv2
import numpy as np
import torch

source_dir = Path(__file__).resolve().parents[2]


def load_frames(limit=20):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag = source_dir / "evidence/rosbags/tesis_point_flow_live_20261123"
    storage = rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    frames = []
    while reader.has_next():
        topic, data, _stamp = reader.read_next()
        if topic != "/camera/d435i/color/image_raw":
            continue
        message = deserialize_message(data, get_message(topic_types[topic]))
        rgb = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width, 3
        )
        if message.encoding == "bgr8":
            rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (256, 192), interpolation=cv2.INTER_AREA)
        frames.append(rgb)
        if len(frames) >= limit:
            break
    return frames


def main():
    weight = Path(os.environ.get("THESIS_RAFT_WEIGHTS", "")).expanduser()
    if not weight.is_file():
        print("Set THESIS_RAFT_WEIGHTS to the local RAFT weight file", file=sys.stderr)
        return 2
    if not torch.cuda.is_available():
        print("CUDA is required", file=sys.stderr)
        return 2

    torch.backends.cudnn.benchmark = False
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False

    from torchvision.models.optical_flow import raft_small

    eager = raft_small(weights=None, progress=False)
    eager.load_state_dict(torch.load(str(weight), map_location="cpu", weights_only=True))
    eager = eager.eval().to("cuda")
    compiled = torch.compile(eager, mode="reduce-overhead")

    frames = load_frames()
    if len(frames) < 2:
        print("Insufficient RGB frames in recorded bag", file=sys.stderr)
        return 2

    max_difference = 0.0
    mean_difference = 0.0
    for previous, current in zip(frames, frames[1:]):
        arrays = np.stack((current, previous, previous, current))
        tensors = (
            torch.from_numpy(arrays)
            .permute(0, 3, 1, 2)
            .contiguous()
            .to("cuda", dtype=torch.float32)
            .div_(127.5)
            .sub_(1.0)
        )
        with torch.inference_mode():
            eager_flow = eager(tensors[:2], tensors[2:], num_flow_updates=12)[-1]
            compiled_flow = compiled(tensors[:2], tensors[2:], num_flow_updates=12)[-1]
        difference = (eager_flow - compiled_flow).abs()
        max_difference = max(max_difference, difference.max().item())
        mean_difference = max(mean_difference, difference.mean().item())

    tolerance = 1e-3
    print(
        f"real_frames={len(frames)} "
        f"max_abs={max_difference:.8f} "
        f"mean_abs={mean_difference:.8f}"
    )
    return 0 if max_difference <= tolerance else 1


if __name__ == "__main__":
    raise SystemExit(main())
