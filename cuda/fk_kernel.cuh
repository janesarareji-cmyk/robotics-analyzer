/**
 * fk_kernel.cuh
 * R5.2: batch TCP forward kinematics kernel (fp32 / fp64). Plain C++ interface,
 * no Eigen: the host side fills FkPathTables from a kin::CompiledModel.
 */
#pragma once

#include <cuda_runtime.h>
#include <cstddef>
#include <vector>

namespace gpu {

constexpr int FK_MAX_JOINTS = 64;
constexpr int FK_THREADS = 256;
/// Values per joint in the device table: pre 12, post 12, axis 3, mult, offset.
constexpr int FK_TAB_PER_JOINT = 29;

/// Active path root -> tip, double precision, 3x4 row-major matrices.
struct FkPathTables {
    int n = 0;                       ///< joints on the active path
    std::vector<int> type;           ///< n: 0 revolute, 1 prismatic, 2 fixed
    std::vector<int> src_q;          ///< n: dof index driving the joint (-1 fixed)
    std::vector<double> mult;        ///< n
    std::vector<double> offset;      ///< n
    std::vector<double> pre;         ///< 12 per joint
    std::vector<double> post;        ///< 12 per joint
    std::vector<double> axis;        ///< 3 per joint (unit)
    std::vector<double> tool;        ///< 12
};

/// Bytes of the device value table / index table for n joints.
size_t fk_tab_bytes(int n, bool fp64);
size_t fk_itab_bytes(int n);

/// Build the device tables. Fixed joints are folded (pre = pre * post, post = I).
/// Layout (SoA, element e of joint j at e * n + j): pre[12], post[12], axis[3],
/// mult, offset, then tool[12] (element e at 29 * n + e). Index table:
/// type[n], src_q[n].
void fk_pack_tables(const FkPathTables& t, bool fp64, std::vector<char>& tab,
                    std::vector<int>& itab);

/// Launch the TCP kernel on `count` samples. q: SoA [n_dof][ld] (T), out: SoA
/// [12][ldo] (T): px py pz r00..r22. Throws gpu::CudaError on launch failure.
void fk_launch(bool fp64, int n, const void* d_tab, const int* d_itab, const void* d_q,
               long long ld, void* d_out, long long ldo, long long count, int grid,
               cudaStream_t stream);

}  // namespace gpu
