"""Compensation model helpers."""

from pathlib import Path

from .hybrid import HybridTorqueCompensator, HybridTorqueEstimate, train_hybrid_compensator
from .mlp import CompensationBundle, CompensationMLP, load_compensation as load_mlp_compensation, train_compensation


def load_compensation(path: str | Path):
    """Load either a hybrid checkpoint or the legacy single-MLP baseline."""
    import torch

    checkpoint = torch.load(Path(path).expanduser(), map_location="cpu", weights_only=False)
    if isinstance(checkpoint, dict) and checkpoint.get("kind") == HybridTorqueCompensator.kind:
        return HybridTorqueCompensator.from_checkpoint(checkpoint)
    return load_mlp_compensation(path)


__all__ = [
    "CompensationBundle",
    "CompensationMLP",
    "HybridTorqueCompensator",
    "HybridTorqueEstimate",
    "load_compensation",
    "load_mlp_compensation",
    "train_compensation",
    "train_hybrid_compensator",
]
