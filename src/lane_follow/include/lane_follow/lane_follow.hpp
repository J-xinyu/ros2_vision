#ifndef LANE_FOLLOW__LANE_FOLLOW_HPP_
#define LANE_FOLLOW__LANE_FOLLOW_HPP_
#include <opencv2/opencv.hpp>

#include <vector>

namespace lane_follow
{
class LaneFollow
{
public:
  cv::Mat preProcess(const cv::Mat & image);
  std::vector<std::vector<cv::Point>> getContours(const cv::Mat & image);
  std::vector<cv::Point> getMaxContour(const cv::Mat & image);

private:
  //   std::vector<int> lower_boundary;
  //   std::vector<int> upper_boundary;
  cv::Scalar lower_boundary;
  cv::Scalar upper_boundary;
  double max_contour_area;
};
}  // namespace lane_follow
#endif  // LANE_FOLLOW__LANE_FOLLOW_HPP_
