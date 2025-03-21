#pragma once

#include "data/vectors.h"
#include <sys/types.h>

namespace newton
{
  struct VelocityCmd
  {
    u_long timestamp;

    Vector3 linear_velocity;
    Vector3 angular_velocity;
  };
} // namespace newton
