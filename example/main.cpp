#include <cubic_spline_generator/spline.hpp>

int main() {
  auto spline = Spline({{0, 0}, {50, 0}, {100, 200}}, 0, 1.57, 100, 100);

  auto seg_num = spline.getSegNum();
  std::cout << seg_num << std::endl;
  for (int i = 0; i < seg_num; i++) {
    for (double s = 0.; s < 1; s += 0.01) {
      auto point = spline.getPoint(i, s);
      std::cout << point[0] << " " << point[1] << std::endl;
    }
  }
  return 0;
}

