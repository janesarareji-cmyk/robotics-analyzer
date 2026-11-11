#pragma once
#include <string>
#include <array>

namespace urm {

/// Inertial properties of a link (for dynamics — R6+).
struct Inertia {
    double mass = 1.0;                       ///< [kg]
    std::array<double,3> com = {0,0,0};      ///< Centre of mass [m]
    std::array<double,6> inertia_tensor = {1,0,0,1,0,1}; ///< Ixx,Ixy,Ixz,Iyy,Iyz,Izz [kg·m²]
};

/// One link in the kinematic chain.
struct Link {
    std::string name;
    std::string geometry_file;  ///< Path to STL/OBJ (optional, empty = primitive)
    Inertia     inertia;
};

} // namespace urm
