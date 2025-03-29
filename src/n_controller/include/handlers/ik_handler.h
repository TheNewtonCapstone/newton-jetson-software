#pragma once

#include <math.h>
#include <result.h>

#include <memory>
#include <vector>

#include "data/vectors.h"

static float distance(const newton::Vector3 &a, const newton::Vector3 &b);

struct IkSegment
{
  newton::Vector3 position;
  float length;
  float angle;
};

struct IkLeg
{
  std::vector<std::shared_ptr<IkSegment>> segments;

  int segment_count;
  float total_length;
};

class IkSolver
{
public:
  result<void> solve(const size_t leg_idx, const newton::Vector3 target);
  result<void> update_angles();

  inline result<IkLeg> get_leg(const size_t leg_idx)
  {
    if (leg_idx < 0 || leg_idx >= m_legs.size())
    {
      return result<IkLeg>::error("Invalid leg index.");
    }
    return result<IkLeg>::success(m_legs[leg_idx]);
  }
  inline void add_leg(const IkLeg &leg) { m_legs.push_back(leg); }

  inline void set_verbose(bool verbose) { m_verbose = verbose; }
  inline void set_tolerance(float tolerance) { m_tolerance = tolerance; }
  inline void set_max_iterations(int max_iterations)
  {
    m_max_iterations = max_iterations;
  }

private:
  std::vector<IkLeg> m_legs = {};

  bool m_verbose = false;
  float m_tolerance = 0.01f;
  int m_max_iterations = 1000;

  result<void> solve_(IkLeg &leg, const newton::Vector3 target);
};