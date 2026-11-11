#pragma once
#include <string>
#include <array>

namespace urm {

/// Inertial properties of a link (for dynamics — R6+).
struct Inertia {
    double mass = 0.0;                       ///< [kg]; 0 = not set
    std::array<double,3> com = {0,0,0};      ///< Centre of mass [m]
    std::array<double,6> inertia_tensor = {0,0,0,0,0,0}; ///< Ixx,Ixy,Ixz,Iyy,Iyz,Izz about COM, link axes [kg·m²]
};

/// One link in the kinematic chain.
struct Link {
    std::string name;
    std::string geometry_file;  ///< Path to STL/OBJ (optional, empty = primitive)
    Inertia     inertia;
};

} // namespace urm
