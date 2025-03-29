#include "gaits/ik_gait.h"

#include <rclcpp/rclcpp.hpp>

#include "logger.h"

using namespace newton;

IkGait::IkGait(const rclcpp::NodeOptions &options)
    : BaseGait("ik_gait", true, options)
{
  Logger::get_instance().set_logfile("ik_gait.csv");

  std::string log_title = "time,";
  log_title += "hfe_offset,kfe_offset,";
  log_title +=
      "target_fl_hfe,target_fl_kfe,target_fr_hfe,target_fr_kfe,target_hl_hfe,"
      "target_hl_kfe,target_hr_hfe,target_hr_kfe,";
  log_title +=
      "pos_fl_hfe,pos_fl_kfe,pos_fr_hfe,pos_fr_kfe,pos_hl_hfe,pos_hl_kfe,pos_"
      "hr_hfe,pos_hr_kfe,";
  log_title +=
      "vel_fl_hfe,vel_fl_kfe,vel_fr_hfe,vel_fr_kfe,vel_hl_hfe,vel_hl_kfe,vel_"
      "hr_hfe,vel_hr_kfe,";
  log_title +=
      "tor_fl_hfe,tor_fl_kfe,tor_fr_hfe,tor_fr_kfe,tor_hl_hfe,tor_hl_kfe,tor_"
      "hr_hfe,tor_hr_kfe,";
  Logger::INFO("ik_gait", log_title.c_str());

  m_ik_solver = IkSolver();
  m_ik_solver.set_verbose(true);

  const auto NUM_LEGS = 4;
  const auto NUM_SEGMENTS = 2;
  const std::array<float, NUM_SEGMENTS> SEGMENT_LENGTHS = {
      18.0f,
      16.0f,
  }; // Example lengths for segments

  // Create legs and segments
  for (int i = 0; i < NUM_LEGS; ++i)
  {
    IkLeg leg = {
        .segment_count = NUM_SEGMENTS,
    };

    // z is not used in this implementation
    Vector3 current_position = {0.0f, 0.0f, 0.0f};

    for (int j = 0; j < NUM_SEGMENTS; ++j)
    {
      auto segment = std::make_shared<IkSegment>();
      segment->position = current_position;
      segment->length = SEGMENT_LENGTHS[j];
      segment->angle = 0.0f;

      leg.segments.push_back(segment);
      leg.total_length += segment->length;

      // Update the current position for the next segment
      current_position.x += 0.0f;
      current_position.y += -segment->length;
    }

    auto segment = std::make_shared<IkSegment>();
    segment->position = current_position;
    segment->length = 0.0f; // End effector segment
    leg.segments.push_back(segment);

    m_ik_solver.add_leg(leg);
  }

  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    standing_positions[i] = 0.0f;
  }

  BaseGait::init();
};

result<void> IkGait::move()
{
  std::string log_line = "";

  auto now = this->get_clock()->now();
  log_line += std::to_string(now.nanoseconds()) + ",";
  auto current_time = now.seconds();

  std::array<float, NUM_JOINTS> positions{};

  // Calculate target positions for all joints
  int idx = 0;
  for (auto &leg : leg_ids)
  {
    std::string leg_name = leg.first;
    std::array<int, 2> ids = leg.second;

    // Solve IK for the leg
    auto target = Vector3{0.0f, -20.0f, 5.0f};

    m_ik_solver.solve(idx, target);
    auto leg_result = m_ik_solver.get_leg(idx);
    if (leg_result.has_error())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to solve IK for leg %s",
                   leg_name.c_str());
    }

    idx = (idx + 1) % 2; // 2 segments per leg
  }

  // Update angles for all legs
  m_ik_solver.update_angles();

  for (int i = 0; i < NUM_JOINTS; i++)
  {
    auto leg_angle = m_ik_solver.get_leg(i).get_value().segments[i % 2]->angle;

    positions[i] = leg_angle;
    RCLCPP_INFO(this->get_logger(), "Leg %d angle: %f", i, leg_angle);
  }

  // Log target positions
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    log_line += std::to_string(positions[i]) + ",";
  }

  // Send target positions to motors
  set_joints_position(positions);

  // Write to log file
  Logger::INFO("harmonic_gait", log_line.c_str());

  return result<void>::success();
}