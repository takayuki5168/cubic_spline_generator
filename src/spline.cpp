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
    if (calcLength(ref_seg_num, next_s) < ref_length) {
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
  const double delta_s_rate = 0.8;
  double accumulated_length;
  double diff_length;
  const double e = 0.005; //[m]
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

  bool is_prev_over_flag, is_cur_over_flag;
  while(std::abs(diff_length) > e) {
    //std::cout << "diff_length " << diff_length << std::endl;
    //std::cout << "accumulated_length " << accumulated_length << std::endl;
    //std::cout << "s: " << s << std::endl;
    if (diff_length > 0) {
      is_cur_over_flag = false;
      while (s + delta_s > 1) {
        //std::cout << "s + delta_s" << s + delta_s << std::endl;
        //std::cout << "delta_s" << delta_s << std::endl;
        delta_s *= delta_s_rate;
      }
      //std::cout << "last (s + delta_s)" << s + delta_s << std::endl;
      if (s + delta_s == 1) {
        break;
      } else if (s + delta_s < 1) {
        if (is_prev_over_flag) delta_s *= delta_s_rate;
        s += delta_s;
        accumulated_length += calcMinuteLength(seg_num, s) * delta_s;
        //std::cout << "minuteLength: " << calcMinuteLength(seg_num, s) * delta_s  << std::endl;
      }
    } else {
      is_cur_over_flag = true;
      while (s - delta_s < 0) {
        delta_s *= delta_s_rate;
      }
      if (s == 0) {
        break;
      } else if (s - delta_s > 0) {
        if (!is_prev_over_flag) delta_s *= delta_s_rate;
        s -= delta_s;
        accumulated_length -= calcMinuteLength(seg_num, s) * delta_s;
      }
    }
    diff_length = length - accumulated_length;
    is_prev_over_flag = is_cur_over_flag;
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
  if (ref_s == 0) {
    return 0;
  }
  double length_sum = 0;
  double s_rate = ref_s * 0.001;
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


/*
 * calc nearest point from current piont with forward offset (default 0)
 */
std::pair<int, double> Spline::calcNearest(std::array<double, 2> cur_point, int cur_seg_num, double cur_s) const
{
  const double delta_l = 100; //[m]

  auto delta_minus_l_point = moveLength(cur_seg_num, cur_s, -delta_l);
  auto delta_plus_l_point = moveLength(cur_seg_num, cur_s, delta_l);

  double lo_s = delta_minus_l_point.second + (delta_minus_l_point.first - cur_seg_num);
  double hi_s = delta_plus_l_point.second - (cur_seg_num - delta_plus_l_point.first);

  int updated_cur_seg_num = cur_seg_num;
  double updated_cur_s = cur_s;

  const double phi = (1.0 + std::sqrt(5.0)) / 2.0;
  double s_1 = (lo_s * phi + hi_s) / (1.0 + phi);
  double s_2 = (lo_s + hi_s * phi) / (1.0 + phi);
  const double e = 0.000000001;
  while (hi_s - lo_s > e) {
    std::array<int, 2> seg_num_temp = {updated_cur_seg_num, updated_cur_seg_num};
    std::array<double, 2> s_temp = {s_1, s_2};
    while (s_temp.at(0) < 0) {
      seg_num_temp.at(0)--;
      s_temp.at(0)++;
    }
    while (s_temp.at(1) < 0) {
      seg_num_temp.at(1)--;
      s_temp.at(1)++;
    }
    while (s_temp.at(1) > 1) {
      seg_num_temp.at(1)++;
      s_temp.at(1)--;
    }
    while (s_temp.at(0) > 1) {
      seg_num_temp.at(0)++;
      s_temp.at(0)--;
    }
    /*
    if (s_1 < 0) {
      seg_num_temp.at(0)--;
      s_temp.at(0)++;
      if (s_2 < 0) {
        seg_num_temp.at(1)--;
        s_temp.at(1)++;
      }
    }
    if (s_2 > 1) {
      seg_num_temp.at(1)++;
      s_temp.at(1)--;
      if (s_1 > 1) {
        seg_num_temp.at(0)++;
        s_temp.at(0)--;
      }
    }*/

    auto s_1_pose = getPose(seg_num_temp.at(0), s_temp.at(0));
    auto s_2_pose = getPose(seg_num_temp.at(1), s_temp.at(1));
    double cog_to_s_1 = std::pow(cur_point.at(0) - s_1_pose.at(0), 2) + std::pow(cur_point.at(1) - s_1_pose.at(1), 2);
    double cog_to_s_2 = std::pow(cur_point.at(0) - s_2_pose.at(0), 2) + std::pow(cur_point.at(1) - s_2_pose.at(1), 2);

    if (cog_to_s_1 < cog_to_s_2){
      hi_s = s_2;
      s_2 = s_1;
      s_1 = (lo_s * phi + hi_s) / (1.0 + phi);
    } else {
      lo_s = s_1;
      s_1 = s_2;
      s_2 = (lo_s + hi_s * phi) / (1.0 + phi);
    }
  }

  if (lo_s > 1) {
    if (updated_cur_seg_num == getSegNum() - 1) {
      updated_cur_s = 1;
    } else {
      updated_cur_seg_num++;
      updated_cur_s = lo_s - 1;
    }
  } else if (lo_s < 0) {
    if (updated_cur_seg_num == 0) {
      updated_cur_s = 0;
    } else {
      updated_cur_seg_num--;
      updated_cur_s = lo_s + 1;
    }
  } else {
    updated_cur_s = lo_s;
  }

  //auto forward_offset_point = moveLength(updated_cur_seg_num, updated_cur_s, forward_offset);
  //return getPoint(forward_offset_point.first, forward_offset_point.second);
  return {updated_cur_seg_num, updated_cur_s};
}
