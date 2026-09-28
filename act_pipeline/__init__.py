#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ACT (Action Chunking with Transformers) Pipeline for Bimanual OpenArm (16-DOF) RGB-D
"""

from act_pipeline.config import ModelConfig, TrainConfig, EvalConfig

try:
    from act_pipeline.models.act_model import ACTPolicy
except ImportError:
    ACTPolicy = None

try:
    from act_pipeline.data.dataset import BimanualEpisodicDataset
    from act_pipeline.data.preprocess import preprocess_rgbd
    from act_pipeline.data.normalization import compute_norm_stats, save_norm_stats, load_norm_stats
except ImportError:
    BimanualEpisodicDataset = None
    preprocess_rgbd = None
    compute_norm_stats = None
    save_norm_stats = None
    load_norm_stats = None

try:
    from act_pipeline.utils.logger import MetricLogger
    from act_pipeline.utils.checkpoint import CheckpointManager
    from act_pipeline.utils.temporal_ensemble import TemporalEnsemblePolicy
except ImportError:
    MetricLogger = None
    CheckpointManager = None
    TemporalEnsemblePolicy = None

__all__ = [
    "ModelConfig",
    "TrainConfig",
    "EvalConfig",
    "ACTPolicy",
    "BimanualEpisodicDataset",
    "preprocess_rgbd",
    "compute_norm_stats",
    "save_norm_stats",
    "load_norm_stats",
    "MetricLogger",
    "CheckpointManager",
    "TemporalEnsemblePolicy",
]
