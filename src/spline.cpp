#include <cubic_spline_generator/spline.hpp>

std::array<double, 2> Spline::getPoint(int seg_num, double s) const
{
  if (seg_num < 0 or seg_num > via_pos_vec_.size() - 1) {
    std::cout << "seg_num is out of range." << std::endl;
    return {-1, -1};
  }
  if (s < 0 or s > 1) {
    std::cout << "s is out of range." << std::endl;
    return {-1, -1};
  }
    
  double x = a_[0][seg_num] + b_[0][seg_num] * s + c_[0][seg_num] * s * s + d_[0][seg_num] * s * s * s;
  double y = a_[1][seg_num] + b_[1][seg_num] * s + c_[1][seg_num] * s * s + d_[1][seg_num] * s * s * s;
  return {x, y};
}

void Spline::generateTrajectory()
{
  const auto seg_num = via_pos_vec_.size();

  std::array<double, 2> alpha, beta;
  alpha[0] = start_vel_ * std::cos(start_angle_);
  alpha[1] = start_vel_ * std::sin(start_angle_);
  beta[0] = goal_vel_ * std::cos(goal_angle_);
  beta[1] = goal_vel_ * std::sin(goal_angle_);

  if (seg_num == 1) {
    for (auto xy = 0; xy <= 1; xy++) {
      a_[xy].resize(1);
      b_[xy].resize(1);
      c_[xy].resize(1);
      d_[xy].resize(1);

      a_[xy][0] = via_pos_vec_[0][xy];
      b_[xy][0] = alpha[xy];
      c_[xy][0] = 3 * (via_pos_vec_[1][xy] - via_pos_vec_[0][xy])
	- beta[xy] - 2 * alpha[xy];
      d_[xy][0] = 2 * (via_pos_vec_[0][xy] - via_pos_vec_[1][xy])
	+ alpha[xy] + beta[xy];
    }
  } else {
    const auto& sp = via_pos_vec_;
    for (auto xy = 0; xy <= 1; xy++) {
      a_[xy].resize(seg_num);
      b_[xy].resize(seg_num);
      c_[xy].resize(seg_num);
      d_[xy].resize(seg_num);

      std::vector<double> k(seg_num);
      std::vector<double> tmp(seg_num);
      k[0] = -3 * alpha[xy] + 3 * (sp[1][xy] - sp[0][xy]);
      for (size_t i = 1; i < seg_num - 1; i++) {
	k[i] = 3 * (sp[i + 1][xy] - 2 * sp[i][xy] + sp[i - 1][xy]);
      }
      k[seg_num - 1] = 9 * (sp[seg_num][xy] - sp[seg_num - 1][xy])
	- 6 * (sp[seg_num - 1][xy] - sp[seg_num - 2][xy])
	- 3 * beta[xy];

      k[0] /= 2;
      tmp[0] = 0.5f;
      for (size_t i = 1; i < seg_num - 1; i++) {
	k[i] -= k[i - 1];
	tmp[i] = 1 / (4 - tmp[i - 1]);
	k[i] *= tmp[i];
      }
      k[seg_num - 1] -= 2 * k[seg_num - 2];
      tmp[seg_num - 1] = 1 / (7 - 2 * tmp[seg_num - 2]);
      k[seg_num - 1] *= tmp[seg_num - 1];

      for (int i = static_cast<int>(seg_num - 2); i >= 0; i--) {
	k[i] -= tmp[i] * k[i + 1];
      }

      for (size_t i = 0; i < seg_num; i++) {
	c_[xy][i] = k[i];
      }

      // a
      for (size_t i = 0; i < seg_num; i++) {
	a_[xy][i] = sp[i][xy];
      }

      // d
      for (size_t i = 0; i < seg_num - 1; i++) {
	d_[xy][i] = (c_[xy][i + 1] - c_[xy][i]) / 3;
      }
      d_[xy][seg_num - 1]
	= (beta[xy] - sp[seg_num][xy]
	   + sp[seg_num - 1][xy] - c_[xy][seg_num - 1])
	/ 2;

      // b
      for (size_t i = 0; i < seg_num - 1; i++) {
	b_[xy][i] = (sp[i + 1][xy] - sp[i][xy])
	  - (c_[xy][i + 1] + 2 * c_[xy][i]) / 3;
      }
      b_[xy][seg_num - 1]
	= sp[seg_num][xy] - sp[seg_num - 1][xy] - c_[xy][seg_num - 1]
	- (beta[xy] - sp[seg_num][xy]
	   + sp[seg_num - 1][xy] - c_[xy][seg_num - 1])
	/ 2;
    }
  }
}
