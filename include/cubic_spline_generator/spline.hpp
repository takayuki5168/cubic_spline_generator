#pragma once

#include <iostream>
#include <cmath>
#include <array>
#include <vector>

class Spline{
public:
  explicit Spline(std::vector<std::array<double, 2>> via_pos_vec, double start_angle, double goal_angle, double start_vel=5, double goal_vel=5)
    : via_pos_vec_(via_pos_vec),
      start_angle_(start_angle), goal_angle_(goal_angle),
      start_vel_(start_vel), goal_vel_(goal_vel)
  {
    generateTrajectory();
  }

  int getSegNum() const { return via_pos_vec_.size() - 1; }
  std::array<double, 2> getPoint(int seg_num, double s) const;

private:
  std::vector<std::array<double, 2>> via_pos_vec_;
  double start_angle_, goal_angle_;
  double start_vel_, goal_vel_;
  
  std::array<std::vector<double>, 2> a_;
  std::array<std::vector<double>, 2> b_;
  std::array<std::vector<double>, 2> c_;
  std::array<std::vector<double>, 2> d_;

  void generateTrajectory();
};
