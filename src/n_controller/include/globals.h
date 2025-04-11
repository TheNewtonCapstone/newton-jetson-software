#pragma once

#include <array>

namespace newton {
static constexpr int NUM_JOINTS = 8;

static constexpr int NUM_OBSERVATIONS = 33;
static constexpr int NUM_ACTIONS = 8;

static constexpr int ANG_VEL_IDX = 0;
static constexpr int PROJ_GRAV_IDX = 3;
static constexpr int CMD_VEL_IDX = 6;
static constexpr int POSITION_IDX = 9;
static constexpr int VELOCITY_IDX = POSITION_IDX + NUM_JOINTS;
static constexpr int PREV_ACTION_IDX = VELOCITY_IDX + NUM_JOINTS;

static constexpr std::array<const char*, NUM_JOINTS> joint_names = {
    "fl_hfe", "fl_kfe",  // front left
    "fr_hfe", "fr_kfe",  // front right
    "hl_hfe", "hl_kfe",  // hind left
    "hr_hfe", "hr_kfe",  // hind right
};

static constexpr std::array<float, NUM_JOINTS> standing_positions = {
    0.8, -1.4,  // front left
    0.8, -1.4,  // front right
    0.8, -1.4,  // hind left
    0.8, -1.4,  // hind right
};
}  // namespace newton