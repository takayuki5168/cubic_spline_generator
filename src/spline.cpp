#include <cubic_spline_generator/spline.hpp>

std::array<double, 2> Spline::getPoint(int seg_num, double s) const
{
  if (seg_num < 0 or seg_num > via_pos_vec_.size() - 1 - 1) {
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

std::array<double, 3> Spline::getPose(int seg_num, double s) const
{
  if (seg_num < 0 or seg_num > via_pos_vec_.size() - 1 - 1) {
    std::cout << "seg_num is out of range." << std::endl;
    return {-1, -1, -1};
  }
  if (s < 0 or s > 1) {
    std::cout << "s is out of range." << std::endl;
    return {-1, -1, -1};
  }
  
  double dx = b_[0][seg_num] + 2 * c_[0][seg_num] * s + 3 * d_[0][seg_num] * s * s;
  double dy = b_[1][seg_num] + 2 * c_[1][seg_num] * s + 3 * d_[1][seg_num] * s * s;  
    
  double x = a_[0][seg_num] + b_[0][seg_num] * s + c_[0][seg_num] * s * s + d_[0][seg_num] * s * s * s;
  double y = a_[1][seg_num] + b_[1][seg_num] * s + c_[1][seg_num] * s * s + d_[1][seg_num] * s * s * s;
  double theta = std::atan2(dy, dx);
  return {x, y, theta};
}

void Spline::generateTrajectory()
{
  const auto seg_num = via_pos_vec_.size() - 1;

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

std::pair<int, double> Spline::moveLength(int seg_num, double s, double length) const
{
  double all_length = calcLength(seg_num, s) + length;
  if (all_length < 0) { return {0, 0}; }
    
  double ref_seg_num = -1, ref_length;
  double sum_length = 0, previous_sum_length = 0;
  for (int i = 0; i < getSegNum(); i++) {
    sum_length += calcLength(i, 1);
    if (sum_length > all_length) {
      ref_seg_num = i;
      ref_length = all_length - previous_sum_length;
      break;
    }
    previous_sum_length = sum_length;
  }
  if (ref_seg_num < 0) { return {getSegNum() - 1, 1}; }

  // ref_seg_numのref_lengthであるsを探索
  double low_s = 0.0, high_s = 1.0;
  while (high_s - low_s > 1e-5) {
    double next_s = (low_s + high_s) / 2;
    if (calcLength(ref_seg_num, next_s) < length) {
      low_s = next_s;
    } else {
      high_s = next_s;
    }
  }

  return {ref_seg_num, (high_s + low_s) / 2};
}

std::array<double, 2> Spline::convertLength_to_s(int seg_num, double s, double length) const
{
  double delta_s = 0.01;
  const double delta_s_rate = 0.1;
  double accumulated_length;
  double diff_length;
  const double e = 0.00001; //[m]
  bool prev_is_over_flag, cur_is_over_flag;

  double des_length = getLength(seg_num, s) + length;
  //std::cout << "init_diff_length" << des_length << std::endl;  

  if (des_length < 0) {
    if (seg_num == 0) {
      return {0,0};
    } else {
      seg_num -= 1;
      s = 1;
      diff_length = des_length - 1;
    }
  } else if (des_length > getLength(seg_num)) {
    if (seg_num == getSegNum()-1) {
      return {double(seg_num), 1};
    } else {
      seg_num += 1;
      s = 0;
      diff_length = des_length;
    }
  } else {
    diff_length = length;
  }
  //std::cout << "before loop" << std::endl;  
  while(std::abs(diff_length) > e) {
    std::cout << ""; // need to end loop  
    //std::cout << "diff_length " << diff_length << std::endl;
    //std::cout << "accumulated_length " << accumulated_length << std::endl;
    //std::cout << "prev_is_over_flag: " << prev_is_over_flag << std::endl;
    if (diff_length >= 0) {
      if (prev_is_over_flag) delta_s *= delta_s_rate;
      s += delta_s;
      if (s > 1) {
	s = 1;
	cur_is_over_flag = true;
      } else {
	cur_is_over_flag = false;
      }  
      accumulated_length += calcMinuteLength(seg_num, s) * delta_s;
    } else {
      if (!prev_is_over_flag) delta_s *= delta_s_rate;
      s -= delta_s;
      if (s < 0) {
	s = 0;
	cur_is_over_flag = false;
      } else {
	cur_is_over_flag = true;
      }
      accumulated_length -= calcMinuteLength(seg_num, s) * delta_s;
    }
    prev_is_over_flag = cur_is_over_flag;
    diff_length = length - accumulated_length;
  }
  //std::cout << "end loop" << std::endl;  
  return {double(seg_num), s};
}

void Spline::calcLength()
{
  const auto seg_num = via_pos_vec_.size() - 1;
  length_.resize(seg_num);

  for (int i = 0; i < length_.size(); i++) {
    length_.at(i) = 0;
    for (double s = 0; s < 1; s += 0.01) {
      length_.at(i) += calcMinuteLength(i, s) * 0.01;
    }
  }
}

double Spline::calcLength(int seg_num, double ref_s) const
{
  double length_sum = 0;
  const double s_rate = 0.001; 
  for (double s = 0; s < ref_s; s += s_rate) {
    length_sum += calcMinuteLength(seg_num, s) * s_rate;
  }
  return length_sum;
}

double Spline::calcMinuteLength(int seg_num, double s) const
{
  double x = b_[0][seg_num] + 2 * c_[0][seg_num] * s + 3 * d_[0][seg_num] * s * s;
  double y = b_[1][seg_num] + 2 * c_[1][seg_num] * s + 3 * d_[1][seg_num] * s * s;
  double l = std::sqrt(x * x + y * y);
  return l;
}
