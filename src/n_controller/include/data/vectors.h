#pragma once

namespace newton {
struct Vector3 {
  float x;
  float y;
  float z;

  Vector3 operator +=(const Vector3 &rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }

  Vector3 operator *(const double &rhs) {
    x *= rhs;
    y *= rhs;
    z *= rhs;
    return *this;
  }
};
}  // namespace newton