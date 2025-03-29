#include "data/vectors.h"
#include <cmath>
#include <sys/types.h>

namespace newton {
struct ImuReading {
  u_long timestamp;

  Vector3 linear_acceleration;

  Vector3 linear_velocity;
  Vector3 angular_velocity;

  Vector3 rotation;
  Vector3 projected_gravity;
};

inline Vector3 quat_to_rpy(const double q, const double x, const double y,
                           const double z) {
  Vector3 rpy;

  rpy.x = atan2(2.0 * (q * x + y * z), 1 - 2 * (x * x + y * y));
  rpy.y = asin(2.0 * (q * y - z * x));
  rpy.z = atan2(2.0 * (q * z + x * y), 1 - 2 * (y * y + z * z));

  return rpy;
}

inline Vector3 quat_to_proj_gravity(const double q, const double x,
                                    const double y, const double z) {
  Vector3 proj_gravity;

  proj_gravity.x = -2.0 * (x * z - q * y);
  proj_gravity.y = -2.0 * (y * z + q * x);
  proj_gravity.z = -(q * q - x * x - y * y + z * z);

  return proj_gravity;
}
} // namespace newton
