"""
Module: math_utils.py
Project: Time-Optimal Control for Quadrotor
"""

import numpy as np

def safe_norm(vec):
  if vec is None:
    return np.nan
  return np.linalg.norm(vec)