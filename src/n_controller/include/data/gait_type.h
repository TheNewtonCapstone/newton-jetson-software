#pragma once
#include <string>

namespace newton {
enum class GaitType { STANDING, WALKING, SLIDING, HARMONIC, MACHINE_LEARNING };

GaitType string_to_gait_type(const std::string &gait_name) {
  if (gait_name == "walking") return GaitType::WALKING;
  if (gait_name == "sliding") return GaitType::SLIDING;
  if (gait_name == "harmonic") return GaitType::HARMONIC;
  if (gait_name == "machine" || gait_name == "rl")
    return GaitType::MACHINE_LEARNING;
  return GaitType::STANDING;  // Default
}
std::string to_str(GaitType type) {
  switch (type) {
    case GaitType::WALKING:
      return "walking";
    case GaitType::SLIDING:
      return "sliding";
    case GaitType::HARMONIC:
      return "harmonic";
    case GaitType::MACHINE_LEARNING:
      return "machine";
    default:
      return "standing";
  }
}
}  // namespace newton
