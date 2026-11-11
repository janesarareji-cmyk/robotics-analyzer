/**
 * ik_kernel.cuh
 * R5.4 part B: PROTOTYPE fp32 batch damped-least-squares IK kernel (one thread per
 * target). Behind the flag GPU_IK_EXPERIMENTAL: the compute backend does not use it
 * (see docs/gpu-ik-decision.md). Plain C++ interface, no Eigen.
 * Work happens in the BASE frame (host converts world targets with the inverse base).
 */
#pragma once

#include <cuda_runtime.h>

namespace gpu {

constexpr int IK_THREADS = 128;
constexpr int IK_MAX_DOF = 12;

/// Status codes (same values as kin::IkStatus).
constexpr int IKS_CONVERGED = 0, IKS_MAX_ITER = 1, IKS_STALLED = 2, IKS_NAN = 3;

/// Kernel parameters, passed by value.
struct IkParams {
    int nd = 0;            ///< dof count (<= IK_MAX_DOF)
    int max_iter = 100;
    float w[6] = {1, 1, 1, 1, 1, 1};  ///< row weights x y z rx ry rz (0 = ignore row)
    float tol_pos = 1e-4f;
    float tol_rot = 1e-4f;
    float lam0 = 0.05f;    ///< damping when the error is large
    float lam_min = 5e-3f; ///< fp32 lower bound
    float lo[IK_MAX_DOF] = {};
    float hi[IK_MAX_DOF] = {};
    float cap[IK_MAX_DOF] = {};  ///< per-dof step cap: 0.2 rad, 0.05 m prismatic
};

/// Device arrays, SoA with leading dimension ld (>= count):
///   tgt  [12][ld]  base-frame target: px py pz r00..r22
///   q0   [nd][ld]
///   qout [nd][ld], st [ld] (IKS_*), perr [ld], rerr [ld], iters [ld]
/// Tables as in fk_kernel.cuh (float). Throws gpu::CudaError on launch failure.
void ik_launch(int n, const void* d_tab, const int* d_itab, const IkParams& prm,
               const float* d_tgt, const float* d_q0, long long ld, float* d_qout, int* d_st,
               float* d_perr, float* d_rerr, int* d_iters, long long count,
               cudaStream_t stream);

}  // namespace gpu
