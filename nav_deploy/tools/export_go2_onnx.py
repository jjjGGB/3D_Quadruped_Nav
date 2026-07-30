#!/usr/bin/env python3
"""Export a SEA-Nav Go2 training checkpoint to a deployment ONNX model."""

import argparse
import copy
import hashlib
import inspect
import json
import math
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn


HISTORY_LENGTH = 10
OBSERVATION_STEP_SIZE = 55
OBSERVATION_SIZE = HISTORY_LENGTH * OBSERVATION_STEP_SIZE


class ExactLSECBFLayer(nn.Module):
    """Closed-form safety layer used by the SEA-Nav training policy."""

    def __init__(
        self,
        num_rays=41,
        fov_deg=180.0,
        safe_radius=0.15,
        safety_margin=0.05,
        kappa=10.0,
        damping_factor=1.0,
    ):
        super().__init__()
        self.d_safe = safe_radius + safety_margin
        self.kappa = kappa
        self.damping_factor = damping_factor

        half_fov = math.radians(fov_deg) / 2.0
        angles = torch.linspace(-half_fov, half_fov, num_rays)
        self.register_buffer(
            "ray_unit_vectors",
            torch.stack((torch.cos(angles), torch.sin(angles)), dim=1),
        )

    def forward(self, nominal_action, lidar_distance, alpha):
        nominal_xy = nominal_action[:, :2]
        yaw_rate = nominal_action[:, 2:]
        barrier_per_ray = lidar_distance - self.d_safe

        min_barrier, _ = torch.min(barrier_per_ray, dim=1, keepdim=True)
        composite_barrier = min_barrier - (1.0 / self.kappa) * torch.log(
            torch.sum(
                torch.exp(
                    -self.kappa * (barrier_per_ray - min_barrier)
                ),
                dim=1,
                keepdim=True,
            )
        )
        ray_weight = torch.exp(
            -self.kappa * (barrier_per_ray - composite_barrier)
        ).unsqueeze(-1)
        barrier_gradient = -torch.sum(
            ray_weight * self.ray_unit_vectors.unsqueeze(0), dim=1
        )

        gradient_action = torch.sum(
            barrier_gradient * nominal_xy, dim=1, keepdim=True
        )
        gradient_norm_squared = torch.sum(
            barrier_gradient**2, dim=1, keepdim=True
        )
        correction_scale = -(
            gradient_action + alpha * composite_barrier
        ) / (gradient_norm_squared + self.damping_factor)
        safe_xy = (
            nominal_xy
            + torch.nn.functional.relu(correction_scale)
            * barrier_gradient
        )
        return torch.cat((safe_xy, yaw_rate), dim=-1)


class Go2CheckpointPolicy(nn.Module):
    """Self-contained architecture matching the Go2 training checkpoint."""

    def __init__(self):
        super().__init__()
        self.num_actions = 3
        self.num_props = 12
        self.num_rays = 41
        self.num_obs_one_step = OBSERVATION_STEP_SIZE
        self.num_latent = 16

        self.backbone = self._mlp((71, 512, 256, 128), final_elu=True)
        self.nav_head = self._mlp((128, 128, 3))
        self.critic = self._mlp((71, 512, 256, 128, 1))
        self.encoder = self._mlp((OBSERVATION_SIZE, 512, 256, 128, 16))
        self.alpha_head = self._mlp((128, 64, 1))
        self.cbf_layer = ExactLSECBFLayer(num_rays=self.num_rays)
        self.std = nn.Parameter(1.5 * torch.ones(self.num_actions))

    @staticmethod
    def _mlp(dimensions, final_elu=False):
        layers = []
        for index, (input_size, output_size) in enumerate(
            zip(dimensions, dimensions[1:])
        ):
            layers.append(nn.Linear(input_size, output_size))
            if index < len(dimensions) - 2 or final_elu:
                layers.append(nn.ELU())
        return nn.Sequential(*layers)

    def forward(self, observation_history):
        latest = observation_history[:, -self.num_obs_one_step:]
        rays_log2 = latest[
            :, self.num_props:self.num_props + self.num_rays
        ]
        latent = self.encoder(observation_history)
        shared = self.backbone(torch.cat((latest, latent.detach()), dim=-1))
        nominal_action = self.nav_head(shared)
        alpha = torch.nn.functional.softplus(self.alpha_head(shared))
        return self.cbf_layer(
            nominal_action, torch.exp2(rays_log2), alpha
        )


class Go2OnnxPolicy(nn.Module):
    """Inference-only graph matching the trained actor and safety layer."""

    def __init__(self, actor_critic):
        super().__init__()
        # The critic and exploration standard deviation are training-only.
        self.encoder = copy.deepcopy(actor_critic.encoder)
        self.backbone = copy.deepcopy(actor_critic.backbone)
        self.nav_head = copy.deepcopy(actor_critic.nav_head)
        self.alpha_head = copy.deepcopy(actor_critic.alpha_head)
        self.cbf_layer = copy.deepcopy(actor_critic.cbf_layer)
        self.num_obs_one_step = actor_critic.num_obs_one_step
        self.num_props = actor_critic.num_props
        self.num_rays = actor_critic.num_rays

    def forward(self, observation_history):
        latest = observation_history[:, -self.num_obs_one_step:]
        rays_log2 = latest[:, self.num_props:self.num_props + self.num_rays]

        latent = self.encoder(observation_history)
        shared = self.backbone(torch.cat((latest, latent), dim=-1))
        nominal_action = self.nav_head(shared)
        alpha = torch.nn.functional.softplus(self.alpha_head(shared))

        # Training stores log2(distance). Using exp(x * ln(2)) instead of exp2
        # keeps the same value while mapping to standard ONNX Mul + Exp ops.
        ray_distance = torch.exp(rays_log2 * math.log(2.0))
        return self.cbf_layer(nominal_action, ray_distance, alpha)


def default_checkpoint():
    return (
        Path(__file__).resolve().parents[1] / "models/model_2000.pt"
    )


def build_policy(checkpoint_path):
    actor_critic = Go2CheckpointPolicy()
    try:
        checkpoint = torch.load(
            str(checkpoint_path), map_location="cpu", weights_only=True
        )
    except TypeError:
        checkpoint = torch.load(str(checkpoint_path), map_location="cpu")
    actor_critic.load_state_dict(checkpoint["model_state_dict"], strict=True)
    actor_critic.eval()
    return (
        actor_critic,
        Go2OnnxPolicy(actor_critic).eval(),
        checkpoint.get("iter"),
    )


def realistic_observation(batch_size, seed=1):
    """Generate bounded inputs covering every observation field."""
    rng = np.random.default_rng(seed)
    observation = np.zeros(
        (batch_size, HISTORY_LENGTH, OBSERVATION_STEP_SIZE), dtype=np.float32
    )
    observation[:, :, 2] = -1.0
    observation[:, :, 6:12] = rng.normal(
        0.0, 0.1, (batch_size, HISTORY_LENGTH, 6)
    )
    rays = rng.uniform(
        0.15, 5.0, (batch_size, HISTORY_LENGTH, 41)
    ).astype(np.float32)
    observation[:, :, 12:53] = np.log2(rays)
    observation[:, :, 53] = rng.uniform(
        1.0, 4.0, (batch_size, HISTORY_LENGTH)
    )
    observation[:, :, 54] = rng.uniform(
        -2.0, 2.0, (batch_size, HISTORY_LENGTH)
    )
    return observation.reshape(batch_size, OBSERVATION_SIZE)


def verify_export(original_model, export_model, onnx_path):
    try:
        import onnx
        import onnxruntime as ort
    except ImportError as error:
        raise RuntimeError(
            "Install export dependencies from requirements-export.txt"
        ) from error

    onnx.checker.check_model(onnx.load(str(onnx_path)))
    session = ort.InferenceSession(
        str(onnx_path), providers=["CPUExecutionProvider"]
    )
    input_name = session.get_inputs()[0].name
    max_wrapper_error = 0.0
    max_onnx_error = 0.0

    for batch_size in (1, 4):
        observation = realistic_observation(batch_size, seed=batch_size)
        tensor = torch.from_numpy(observation)
        with torch.no_grad():
            original_action = original_model(tensor).numpy()
            wrapper_action = export_model(tensor).numpy()
        onnx_action = session.run(None, {input_name: observation})[0]

        wrapper_error = float(
            np.max(np.abs(original_action - wrapper_action))
        )
        onnx_error = float(np.max(np.abs(wrapper_action - onnx_action)))
        max_wrapper_error = max(max_wrapper_error, wrapper_error)
        max_onnx_error = max(max_onnx_error, onnx_error)
        if not np.allclose(
            original_action, wrapper_action, rtol=1e-5, atol=1e-6
        ):
            raise RuntimeError(
                f"Export wrapper mismatch; max error={wrapper_error:.6g}"
            )
        if not np.allclose(
            wrapper_action, onnx_action, rtol=1e-4, atol=1e-5
        ):
            raise RuntimeError(
                f"ONNX Runtime mismatch; max error={onnx_error:.6g}"
            )
    return max_wrapper_error, max_onnx_error


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint", type=Path, default=default_checkpoint()
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "models/sea_nav_go2.onnx",
    )
    parser.add_argument("--opset", type=int, default=17)
    return parser.parse_args()


def main():
    args = parse_args()
    checkpoint = args.checkpoint.resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(
            f"Checkpoint not found: {checkpoint}; pass --checkpoint explicitly"
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    original_model, export_model, iteration = build_policy(checkpoint)
    example = torch.from_numpy(realistic_observation(1))
    export_options = {}
    if "dynamo" in inspect.signature(torch.onnx.export).parameters:
        # PyTorch 2.9+ defaults to the onnxscript-based dynamo exporter.
        # The legacy path is stable for this static feed-forward graph and
        # keeps the standalone export dependencies minimal. Older supported
        # PyTorch versions do not expose this option and already use it.
        export_options["dynamo"] = False
    torch.onnx.export(
        export_model,
        example,
        str(args.output),
        input_names=["observation_history"],
        output_names=["navigation_action"],
        dynamic_axes={
            "observation_history": {0: "batch"},
            "navigation_action": {0: "batch"},
        },
        opset_version=args.opset,
        do_constant_folding=True,
        **export_options,
    )
    wrapper_error, onnx_error = verify_export(
        original_model, export_model, args.output
    )

    project_root = Path(__file__).resolve().parents[2]
    try:
        checkpoint_id = str(checkpoint.relative_to(project_root))
    except ValueError:
        checkpoint_id = str(checkpoint)
    model_hash = hashlib.sha256(args.output.read_bytes()).hexdigest()
    metadata = {
        "checkpoint": checkpoint_id,
        "checkpoint_iteration": iteration,
        "onnx_sha256": model_hash,
        "input_name": "observation_history",
        "input_shape": ["batch", OBSERVATION_SIZE],
        "output_name": "navigation_action",
        "output_order": ["vx", "vy", "yaw_rate"],
        "onnx_opset": args.opset,
        "original_wrapper_max_abs_error": wrapper_error,
        "wrapper_onnx_max_abs_error": onnx_error,
    }
    metadata_path = args.output.with_suffix(".json")
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")

    print(f"Exported: {args.output}")
    print(f"Metadata: {metadata_path}")
    print(f"SHA256: {model_hash}")
    print(
        "Max errors: original/wrapper={:.6g}, wrapper/ONNX={:.6g}".format(
            wrapper_error, onnx_error
        )
    )


if __name__ == "__main__":
    main()
