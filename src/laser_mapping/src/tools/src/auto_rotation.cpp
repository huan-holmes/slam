#include "auto_rotation.h"

// opencv
#include <opencv2/imgproc.hpp>
// #include <opencv2/imgcodecs.hpp>

namespace tools {

double AutoRotation::Rotation(int width, int height, double resolution, std::vector<tools::Point>& points) {
  // 填充图像
  cv::Mat img = cv::Mat::zeros(width, height, CV_8U);
  for (const auto& point : points) {
    *img.ptr<uchar>(point.x, point.y) = 255;
  }

  // 调试使用
  cv::Mat color;
  cvtColor(img, color, cv::COLOR_GRAY2BGR);

  // cv::imwrite("/home/anson/temp/1.png", img);

  // 检测直线
  std::vector<cv::Vec4i> lines;

  // 最短直线0.5米
  double min_line_length = 0.5 / resolution;
  // 最大间距20cm
  double max_line_gap = 0.1 / resolution;

  cv::HoughLinesP(img, lines, 1, CV_PI / 180, 80, min_line_length, max_line_gap);

  if (lines.empty()) {
    printf("cannot detect lines\n");
    return 270.0;
  }

  // 画线
  for (const auto& line : lines) {
    cv::line(color, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 3, 8);
  }
  // cv::imwrite("/home/anson/temp/2.png", color);

  // 生成(-180.0, 180.]间隔为step的角度表
  // std::pair<角度, 距离的平方>
  double step = 0.5;
  double step_half = 0.25;
  std::vector<std::pair<double, int>> angle_vec;
  for (double i = -180. + step;;) {
    angle_vec.emplace_back(i, 0.);
    i += 2.0;
    if (i > 180.5) {
      break;
    }
  }

  // 找出最长的直线或者平行直线和最长的
  for (const auto& line : lines) {
    cv::Point p1(line[0], line[1]);
    cv::Point p2(line[2], line[3]);
    auto p = p1 - p2;
    // 计算角度
    auto angle = atan2(p.y, p.x) * 180. / M_PI;
    // 距离的平方
    auto dist_sqr = p.dot(p);

    // 找角度表中最相近的角度

    // 返回第一个不小于当前角度的值
    auto iter = std::lower_bound(angle_vec.begin(), angle_vec.end(), angle,
                                 [](const std::pair<double, double>& l, const double& r) { return l.first < r; });
    if (iter == angle_vec.end()) {
      printf("fatal error\n");
      exit(0);
    }

    // 如果角度大于步长一半，说明属于上一个
    if ((*iter).first - angle > step_half) {
      // 判断是否为第一个
      if (iter == angle_vec.begin()) {
        angle_vec.back().second += dist_sqr;
      } else {
        --iter;
      }
    }

    // 累加距离的平方
    (*iter).second += dist_sqr;
  }

  // 距离的平方最大的角度
  int max_index{0}, max_value{0};
  for (int i = 0; i < angle_vec.size(); i++) {
    if (angle_vec.at(i).second > max_value) {
      max_value = angle_vec.at(i).second;
      max_index = i;
    }
  }

  printf("max index: %d, angle: %.2f, dist: %d\n", max_index, angle_vec.at(max_index).first, max_value);

  // debug
  for (const auto& angle : angle_vec) {
    if (angle.second > 0) {
      printf("%.2f: %d\n", angle.first, angle.second);
    }
  }

  return angle_vec.at(max_index).first;
}
}  // namespace tools
