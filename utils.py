import numpy as np


def dict_to_list(q_dict, ref_dict) -> list:
    q = [0] * 12
    for k, v in q_dict.items():
        idx = ref_dict[k]
        q[idx] = v
    return q


def list_to_dict(q_list, ref_dict) -> dict:
    q = {}
    for k, v in ref_dict.items():
        q[k] = q_list[v]
    return q


def list_to_list(q_list, from_dict, to_dict) -> list:
    q = [0] * 12
    for k, v in from_dict.items():
        idx = to_dict[k]
        q[idx] = q_list[v]
    return q


def linear_interp(
    current: np.ndarray,
    desired: np.ndarray,
    steps: int
) -> np.ndarray:
    alpha = 1.0
    interp = []
    for _ in range(steps):
        interp.append(alpha * current + (1 - alpha) * desired)
        alpha -= 1. / steps
    return np.array(interp)
