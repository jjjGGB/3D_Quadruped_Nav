"""Validate the deployed model interface and run a CPU inference smoke test."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import onnxruntime as ort

from nav_deploy.policy_core import OBSERVATION_SIZE


def default_model_path():
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("nav_deploy")) / "models" / "sea_nav_go2.onnx"
    except Exception:
        return Path(__file__).resolve().parents[1] / "models" / "sea_nav_go2.onnx"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", nargs="?", type=Path, default=default_model_path())
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.model.is_file():
        raise FileNotFoundError(args.model)

    session = ort.InferenceSession(
        str(args.model), providers=["CPUExecutionProvider"]
    )
    model_input = session.get_inputs()[0]
    model_output = session.get_outputs()[0]
    if model_input.name != "observation_history":
        raise RuntimeError(f"Unexpected input name: {model_input.name}")
    if model_input.shape[-1] != OBSERVATION_SIZE:
        raise RuntimeError(f"Unexpected input shape: {model_input.shape}")
    if model_output.name != "navigation_action" or model_output.shape[-1] != 3:
        raise RuntimeError(
            f"Unexpected output: {model_output.name} {model_output.shape}"
        )

    observation = np.zeros((1, OBSERVATION_SIZE), dtype=np.float32)
    one_step = observation.reshape(1, 10, 55)
    one_step[:, :, 2] = -1.0
    one_step[:, :, 12:53] = np.log2(5.0)
    one_step[:, :, 53] = 3.0
    action = session.run(None, {model_input.name: observation})[0]
    if action.shape != (1, 3) or not np.isfinite(action).all():
        raise RuntimeError(f"Invalid inference output: {action}")

    metadata_path = args.model.with_suffix(".json")
    metadata = json.loads(metadata_path.read_text()) if metadata_path.is_file() else {}
    if metadata.get("onnx_sha256"):
        model_hash = hashlib.sha256(args.model.read_bytes()).hexdigest()
        if model_hash != metadata["onnx_sha256"]:
            raise RuntimeError(
                f"Model SHA256 mismatch: {model_hash} != "
                f"{metadata['onnx_sha256']}"
            )
    print(f"PASS: {args.model}")
    print(
        f"interface: {model_input.name} {model_input.shape} -> "
        f"{model_output.name} {model_output.shape}"
    )
    print(f"CPU smoke action [vx, vy, yaw_rate]: {action[0].tolist()}")
    if metadata:
        print(
            "export errors: PyTorch/wrapper={:.6g}, wrapper/ONNX={:.6g}".format(
                metadata["original_wrapper_max_abs_error"],
                metadata["wrapper_onnx_max_abs_error"],
            )
        )


if __name__ == "__main__":
    main()
