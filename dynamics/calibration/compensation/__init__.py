"""Compensation model helpers."""

from pathlib import Path

from dynamics.embodiment import assert_embodiment_matches, build_embodiment_metadata

from .history import (
    KinematicHistoryCompensator,
    KinematicHistoryEstimate,
    train_kinematic_history_model,
)
from .hybrid import HybridTorqueCompensator, HybridTorqueEstimate, train_hybrid_compensator
from .mlp import CompensationBundle, CompensationMLP, load_compensation as load_mlp_compensation, train_compensation


def load_compensation(path: str | Path):
    """Load either a hybrid checkpoint or the legacy single-MLP baseline."""
    import torch

    checkpoint = torch.load(Path(path).expanduser(), map_location="cpu", weights_only=False)
    if isinstance(checkpoint, dict) and checkpoint.get("kind") == HybridTorqueCompensator.kind:
        return HybridTorqueCompensator.from_checkpoint(checkpoint)
    if isinstance(checkpoint, dict) and checkpoint.get("kind") == KinematicHistoryCompensator.kind:
        return KinematicHistoryCompensator.from_checkpoint(checkpoint)
    return load_mlp_compensation(path)


def _checkpoint_embodiment(checkpoint: dict):
    if checkpoint.get("kind") == HybridTorqueCompensator.kind:
        return dict(checkpoint.get("metadata", {})).get("embodiment")
    if checkpoint.get("kind") == KinematicHistoryCompensator.kind:
        return dict(checkpoint.get("metadata", {})).get("embodiment")
    return dict(checkpoint.get("extra", {})).get("embodiment")


def load_compensation_for_config(path: str | Path, config: dict):
    """Load a compensation checkpoint after checking it matches the config embodiment."""
    import torch

    checkpoint = torch.load(Path(path).expanduser(), map_location="cpu", weights_only=False)
    if not isinstance(checkpoint, dict):
        raise ValueError(f"not a compensation checkpoint: {path}")
    expected = build_embodiment_metadata(config)
    assert_embodiment_matches(_checkpoint_embodiment(checkpoint), expected)
    if checkpoint.get("kind") == HybridTorqueCompensator.kind:
        return HybridTorqueCompensator.from_checkpoint(checkpoint)
    if checkpoint.get("kind") == KinematicHistoryCompensator.kind:
        return KinematicHistoryCompensator.from_checkpoint(checkpoint)
    return load_mlp_compensation(path)


__all__ = [
    "CompensationBundle",
    "CompensationMLP",
    "HybridTorqueCompensator",
    "HybridTorqueEstimate",
    "KinematicHistoryCompensator",
    "KinematicHistoryEstimate",
    "load_compensation",
    "load_compensation_for_config",
    "load_mlp_compensation",
    "train_compensation",
    "train_hybrid_compensator",
    "train_kinematic_history_model",
]
