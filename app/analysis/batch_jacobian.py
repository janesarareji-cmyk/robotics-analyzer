"""U6.2: fast batch indicators (sigma_min, manipulability) for millions of joint samples.

``cb.jacobian_tcp`` is one call per sample (Python loop, about 25 us each). For the overlays the
Jacobian is built for a whole batch from ``cb.fk_batch`` (double, multi-threaded) by forward
differences of the TCP pose: column i = ((p(q + e_i) - p(q)) / e, vee(dR R^T) / e), e = 1e-6.
Error about 1e-6 of the Jacobian (checked against ``cb.jacobian_tcp`` in the tests), far below the
thresholds (0.05) the overlays use. Rows 0-2 linear, 3-5 angular, BASE frame (singular values do
not depend on the frame).
"""
from __future__ import annotations

import numpy as np

import core_bridge as cb

EPS = 1e-6


def batch_pose_jacobian(cm, q: np.ndarray, eps: float = EPS):
    """(J (N,6,n), p (N,3), R (N,3,3)) at the samples ``q`` (N, n)."""
    q = np.ascontiguousarray(q, dtype=float)
    n_s, n = q.shape
    f0 = cb.fk_batch(cm, q, "base")
    p0 = np.asarray(f0["position"])
    R0 = np.asarray(f0["rotation"])
    J = np.empty((n_s, 6, n))
    for i in range(n):
        q1 = q.copy()
        q1[:, i] += eps
        f1 = cb.fk_batch(cm, q1, "base")
        J[:, 0:3, i] = (np.asarray(f1["position"]) - p0) / eps
        S = np.matmul(np.asarray(f1["rotation"]) - R0, np.swapaxes(R0, 1, 2)) / eps
        J[:, 3, i] = 0.5 * (S[:, 2, 1] - S[:, 1, 2])
        J[:, 4, i] = 0.5 * (S[:, 0, 2] - S[:, 2, 0])
        J[:, 5, i] = 0.5 * (S[:, 1, 0] - S[:, 0, 1])
    return J, p0, R0


def _gram_eigs(J: np.ndarray, mode: str) -> np.ndarray:
    A = J[:, :3, :] if mode == "position" else J
    m, n = A.shape[1], A.shape[2]
    G = np.matmul(A, np.swapaxes(A, 1, 2)) if m <= n else np.matmul(np.swapaxes(A, 1, 2), A)
    return np.linalg.eigvalsh(G)             # ascending, (N, min(m, n))


def sigma_min_batch(J: np.ndarray, mode: str = "full") -> np.ndarray:
    """Smallest singular value per sample (mode "full": 6 x n, "position": rows 0-2)."""
    ev = _gram_eigs(J, mode)
    return np.sqrt(np.maximum(ev[:, 0], 0.0))


def metrics_batch(J: np.ndarray, mode: str = "full") -> tuple[np.ndarray, np.ndarray]:
    """(sigma_min, Yoshikawa w = sqrt(det(J J^T))) per sample."""
    ev = np.maximum(_gram_eigs(J, mode), 0.0)
    return np.sqrt(ev[:, 0]), np.sqrt(np.prod(ev, axis=1))


def fast_sample_metrics(cm, q: np.ndarray, mode: str = "position", chunk: int = 100_000):
    """Drop-in for ``workspace_analysis.sample_metrics``: ``(sigma_min, w)`` of (N, n) samples."""
    q = np.asarray(q, dtype=float)
    smin = np.empty(len(q))
    w = np.empty(len(q))
    for a in range(0, len(q), chunk):
        J, _p, _R = batch_pose_jacobian(cm, q[a:a + chunk])
        smin[a:a + chunk], w[a:a + chunk] = metrics_batch(J, "position" if mode == "position" else "full")
    return smin, w
