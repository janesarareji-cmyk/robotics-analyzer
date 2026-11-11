#include "kin/singularity.hpp"

#include <limits>
#include <stdexcept>

namespace kin {

SingularityInfo analyze_singularity(const Eigen::MatrixXd& J, const std::string& mode, double threshold) {
    if (J.rows() != 6) {
        throw std::invalid_argument("analyze_singularity: J must have 6 rows");
    }

    int row0 = 0;
    int nrow = 0;
    if (mode == "full") {
        row0 = 0;
        nrow = 6;
    } else if (mode == "position") {
        row0 = 0;
        nrow = 3;
    } else if (mode == "angular") {
        row0 = 3;
        nrow = 3;
    } else {
        throw std::invalid_argument("analyze_singularity: unknown mode '" + mode + "'");
    }

    const Eigen::MatrixXd M = J.middleRows(row0, nrow);

    std::vector<int> cols;
    cols.reserve(static_cast<size_t>(M.cols()));
    for (int c = 0; c < M.cols(); ++c) {
        if ((M.col(c).array() != 0.0).any()) {
            cols.push_back(c);
        }
    }

    SingularityInfo info;
    if (cols.empty()) {
        info.singular_values.assign(static_cast<size_t>(nrow), 0.0);
        info.sigma_min = 0.0;
        info.sigma_max = 0.0;
        info.condition = std::numeric_limits<double>::infinity();
        info.manipulability = 0.0;
        info.near_singular = true;
        return info;
    }

    Eigen::MatrixXd Mnz(nrow, static_cast<Eigen::Index>(cols.size()));
    for (size_t k = 0; k < cols.size(); ++k) {
        Mnz.col(static_cast<Eigen::Index>(k)) = M.col(cols[k]);
    }

    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(Mnz);
    const Eigen::VectorXd sv = svd.singularValues();
    const Eigen::Index r = sv.size();  // min(nrow, nonzero cols)

    info.singular_values.assign(sv.data(), sv.data() + r);
    info.sigma_max = sv(0);
    info.sigma_min = sv(r - 1);
    info.condition = (info.sigma_min < 1e-300)
                         ? std::numeric_limits<double>::infinity()
                         : info.sigma_max / info.sigma_min;
    info.manipulability = sv.prod();
    info.near_singular = info.sigma_min < threshold;
    return info;
}

} // namespace kin
