#pragma once
#include "robot_model.hpp"
#include <string>

namespace urm {

/// Serialize RobotModel to JSON string.
std::string to_json(const RobotModel& model, int indent = 2);

/// Deserialize RobotModel from JSON string.
RobotModel from_json(const std::string& json_str);

} // namespace urm
