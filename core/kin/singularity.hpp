#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace kin {

/// Singularity indicators of a 6 x n Jacobian (R4.5).
struct SingularityInfo {
    std::vector<double> singular_values;  ///< first r values, descending (r = min(rows, nonzero cols))
    double sigma_min = 0.0;               ///< singular_values[r-1]
    double sigma_max = 0.0;               ///< singular_values[0]
    double condition = 0.0;               ///< sigma_max / sigma_min, +inf if sigma_min < 1e-300
    double manipulability = 0.0;          ///< Yoshikawa: product of the r singular values
    bool near_singular = false;           ///< sigma_min < threshold
};

/// Analyze a 6 x n Jacobian J.
/// mode: "full" uses rows 0-5, "position" rows 0-2, "angular" rows 3-5.
/// Other mode -> std::invalid_argument. J must have 6 rows (else invalid_argument).
/// Columns that are exactly zero (DOFs off the active chain) are dropped before the SVD.
/// If no nonzero column remains: singular_values are zeros (one per selected row),
/// condition = +inf, manipulability = 0, near_singular = true.
SingularityInfo analyze_singularity(const Eigen::MatrixXd& J,
                                    const std::string& mode = "full",
                                    double threshold = 1e-3);

} // namespace kin
