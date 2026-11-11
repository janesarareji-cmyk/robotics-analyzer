#pragma once

#include <cstdint>
#include <vector>
#include "kin/compiled_model.hpp"

namespace kin {

/// Frozen layout version. Changing CompiledArrays needs a new version and a
/// review note in docs/compiled-model-layout.md.
inline constexpr int32_t kCompiledLayoutVersion = 1;

/// Flat structure-of-arrays copy of a CompiledModel (R4.1).
/// All arrays are in compiled order (CompiledModel::joints, sorted by child_link).
/// Matrices are row-major 4x4 (16 doubles each, last row 0 0 0 1).
/// Field order and sizes are documented in docs/compiled-model-layout.md.
struct CompiledArrays {
    int32_t n_joints = 0;
    int32_t n_links = 0;
    int32_t n_dof = 0;
    int32_t tcp_link = 0;
    int32_t layout_version = kCompiledLayoutVersion;

    std::vector<int32_t> joint_index;   ///< n
    std::vector<int32_t> parent_link;   ///< n
    std::vector<int32_t> child_link;    ///< n
    std::vector<int32_t> type;          ///< n, urm::JointType as int (Revolute 0, Prismatic 1, Fixed 2)
    std::vector<int32_t> q_index;       ///< n, -1 for fixed and mimic joints
    std::vector<int32_t> src_q_index;   ///< n, -1 for fixed joints

    std::vector<double> pre;            ///< n * 16
    std::vector<double> post;           ///< n * 16
    std::vector<double> axis;           ///< n * 3
    std::vector<double> mult;           ///< n
    std::vector<double> offset;         ///< n

    std::vector<double> base;           ///< 16
    std::vector<double> tool;           ///< 16
};

/// Copy a compiled model into flat arrays. Does not change the model.
CompiledArrays to_arrays(const CompiledModel& cm);

/// Rebuild a compiled model from flat arrays.
/// Throws std::invalid_argument on wrong layout_version, wrong array sizes,
/// or an unknown joint type value.
CompiledModel from_arrays(const CompiledArrays& arr);

} // namespace kin
