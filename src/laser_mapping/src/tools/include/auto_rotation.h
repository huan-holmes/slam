#pragma once
#include <vector>

namespace tools {

struct Point {
  Point(int _x, int _y) : x(_x), y(_y) {}
  int x{0};
  int y{0};
};

class AutoRotation {
 public:
  static double Rotation(int width, int height, double resolution, std::vector<tools::Point>& points);
};
}  // namespace tools
