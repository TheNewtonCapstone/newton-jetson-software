#pragma once
#include <cstring>
#include <cstdint>

struct MotorCommand {
  float velocity;
  float position;
  uint8_t motor_id;

  // Method to deserialize from bytes
  static MotorCommand from_bytes(const uint8_t* buffer) {
    MotorCommand cmd;
    // Carefully reconstruct each field, accounting for memory alignment
    memcpy(&cmd.velocity, buffer, sizeof(float));
    memcpy(&cmd.position, buffer + sizeof(float), sizeof(float));
    cmd.motor_id = buffer[sizeof(float) * 2];
    return cmd;
  }
};