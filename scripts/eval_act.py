#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CLI Entrypoint for ACT Evaluation on OpenArm
"""

import sys
import os

# Đảm bảo đường dẫn module gốc có trong sys.path
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from act_pipeline.eval import parse_args, eval_pipeline

if __name__ == "__main__":
    args = parse_args()
    eval_pipeline(args)
