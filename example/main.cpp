#include <cubic_spline_generator/spline.hpp>

int main() {
  auto spline = Spline({{0, 0}, {1, 1}}, 0, 0, 1, 1);

  auto seg_num = spline.getSegNum();
  std::cout << seg_num << std::endl;
  for (int i = 0; i < seg_num; i++) {
    for (double s = 0.; s < 1; s += 0.01) {
      auto pose = spline.getPose(i, s);
      std::cout << pose[0] << " " << pose[1] << " " << pose[2] << std::endl;
    }
  }
  return 0;
}

