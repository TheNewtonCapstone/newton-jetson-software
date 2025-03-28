#include "handlers/ik_handler.h"

static float distance(const newton::Vector3 &a, const newton::Vector3 &b)
{
  float dx = a.x - b.x;
  float dy = a.y - b.y;
  float dz = a.z - b.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

result<void> IkSolver::solve(const size_t leg_idx,
                             const newton::Vector3 target)
{
  if (leg_idx < 0 || leg_idx >= m_legs.size())
  {
    return result<void>::error("Invalid leg index.");
  }

  auto &leg = m_legs[leg_idx];

  if (m_verbose)
  {
    std::cout << "Solving IK for leg " << leg_idx << " to target: " << target.x
              << ", " << target.y << ", " << target.z << std::endl;
  }

  return solve_(leg, target);
}

result<void> IkSolver::update_angles()
{
  for (auto &leg : m_legs)
  {
    for (size_t i = 0; i < leg.segment_count - 1; ++i)
    {
      auto segment = leg.segments[i];
      auto next_segment = leg.segments[i + 1];

      auto dx = next_segment->position.x - segment->position.x;
      auto dy = next_segment->position.y - segment->position.y;
      auto dz = next_segment->position.z - segment->position.z;

      segment->angle = std::atan2(dy, dx);
    }
  }

  if (m_verbose)
  {
    std::cout << "Updated angles for all legs." << std::endl;
  }

  return result<void>::success();
}

result<void> IkSolver::solve_(IkLeg &leg, const newton::Vector3 target)
{
  // Check if the target is reachable
  auto base_position = leg.segments.front()->position;

  float targetDistance = distance(base_position, target);
  float totalLength = leg.total_length;

  if (targetDistance > totalLength)
  {
    if (m_verbose)
    {
      std::cout << "Target is unreachable." << std::endl;
    }

    // Just stretch in the direction of the target

    // Calculate direction from base to target
    float dx = target.x - base_position.x;
    float dy = target.y - base_position.y;
    float dz = target.z - base_position.z;
    float dist = distance(base_position, target);

    // Normalize direction
    dx /= dist;
    dy /= dist;
    dz /= dist;

    // Position joints along this direction
    leg.segments.front()->position.x = base_position.x;
    leg.segments.front()->position.y = base_position.y;
    leg.segments.front()->position.z = base_position.z;

    for (size_t i = 0; i < leg.segment_count; ++i)
    {
      // Calculate the new position of the joint
      auto current_segment = leg.segments[i];

      current_segment->position.x += current_segment->length * dx;
      current_segment->position.y += current_segment->length * dy;
      current_segment->position.z += current_segment->length * dz;
    }

    return result<void>::error("Target is unreachable.");
  }

  for (int iteration = 0; iteration < m_max_iterations; ++iteration)
  {
    // Current end effector position
    newton::Vector3 current_end_position = leg.segments.back()->position;

    // Check for convergence
    float end_effector_error = distance(current_end_position, target);

    if (end_effector_error < m_tolerance)
    {
      if (m_verbose)
        std::cout << "Converged after " << iteration << " iterations." << std::endl;

      return result<void>::success();
    }

    // --- BACKWARD REACHING ---
    // Set the end effector to the target position
    leg.segments.back()->position = target;

    // Go backward through the joints from the end effector to the root
    for (int i = leg.segment_count - 2; i >= 0; --i)
    {
      auto current_segment = leg.segments[i];
      auto next_segment = leg.segments[i + 1];

      // Direction from current joint to next joint
      float dx = next_segment->position.x - current_segment->position.x;
      float dy = next_segment->position.y - current_segment->position.y;
      float dz = next_segment->position.z - current_segment->position.z;
      float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

      // Normalize direction
      if (dist > 0)
      {
        dx /= dist;
        dy /= dist;
        dz /= dist;
      }

      // Position the current joint at the correct distance from the next joint
      current_segment->position.x =
          next_segment->position.x - current_segment->length * dx;
      current_segment->position.y =
          next_segment->position.y - current_segment->length * dy;
      current_segment->position.z =
          next_segment->position.z - current_segment->length * dz;
    }

    // --- FORWARD REACHING ---
    // Fix the base position
    leg.segments.front()->position = base_position;

    // Go forward through the joints from the root to the end effector
    for (size_t i = 0; i < leg.segment_count - 1; ++i)
    {
      auto current_segment = leg.segments[i];
      auto next_segment = leg.segments[i + 1];

      // Direction from current joint to next joint
      float dx = next_segment->position.x - current_segment->position.x;
      float dy = next_segment->position.y - current_segment->position.y;
      float dz = next_segment->position.z - current_segment->position.z;
      float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

      // Normalize direction
      if (dist > 0)
      {
        dx /= dist;
        dy /= dist;
        dz /= dist;
      }

      // Position the next joint at the correct distance from the current joint
      next_segment->position.x =
          current_segment->position.x + current_segment->length * dx;
      next_segment->position.y =
          current_segment->position.y + current_segment->length * dy;
      next_segment->position.z =
          current_segment->position.z + current_segment->length * dz;
    }

    if (m_verbose)
    {
      std::cout << "Iteration " << iteration << ", Error: "
                << distance(leg.segments.back()->position, target) << std::endl;
    }
  }

  if (m_verbose)
  {
    std::cout << "Maximum iterations reached. Final error: "
              << distance(leg.segments.back()->position, target) << std::endl;
  }

  if (distance(leg.segments.back()->position, target) < m_tolerance)
  {
    return result<void>::success();
  }
  else
  {
    return result<void>::error("Failed to converge within max iterations.");
  }
}
