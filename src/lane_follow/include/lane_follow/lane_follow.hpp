#ifndef LANE_FOLLOW__LANE_FOLLOW_HPP_
#define LANE_FOLLOW__LANE_FOLLOW_HPP_
#include <opencv2/opencv.hpp>

#include <vector>

namespace lane_follow
{
class LaneFollow
{
public:
  LaneFollow(
    const std::vector<int64> & lower_boundary, const std::vector<int64> & upper_boundary,
    const double max_contour_area = 1000.0);
  //   ~LaneFollow();
  cv::Mat preProcess(const cv::Mat & image);
  std::vector<std::vector<cv::Point>> getContours(const cv::Mat & image);
  std::vector<cv::Point> getMaxContour(const cv::Mat & image);

private:
  //   std::vector<int64> lower_boundary_;
  //   std::vector<int64> upper_boundary_;
  cv::Scalar lower_boundary_;
  cv::Scalar upper_boundary_;
  double max_contour_area_;
};
}  // namespace lane_follow
#endif  // LANE_FOLLOW__LANE_FOLLOW_HPP_
