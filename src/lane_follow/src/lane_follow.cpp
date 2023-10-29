#include "lane_follow/lane_follow.hpp"

namespace lane_follow
{
cv::Mat LaneFollow::preProcess(const cv::Mat & image)
{
  cv::Mat hls_img;
  cv::Mat fusion_img;
  cv::Mat binary_img;  // mask
  auto roi_img = image(cv::Rect(0, image.rows / 2, image.rows, image.cols / 2));
  cv::cvtColor(roi_img, hls_img, cv::COLOR_BGR2HLS);
  //   cv::Mat dilate_img;
  //   cv::Mat erode_img;
  //   cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  // 对于本身比较连续的车道线，就先膨胀再腐蚀，用来消除噪点。若车道线有小间隙则先腐蚀后膨胀
  //   cv::dilate(hsv_img, dilate_img, kernel);   // 膨胀
  //   cv::erode(dilate_img, erode_img, kernel);  // 腐蚀
  // 会根据颜色阈值来创建一个mask，然后用这个mask去和原始图片比对得到结果
  cv::inRange(hls_img, lower_boundary, upper_boundary, binary_img);
  auto element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8, 5));
  cv::morphologyEx(binary_img, binary_img, cv::MORPH_CLOSE, element);  // 闭运算就是先膨胀后腐蚀

  cv::bitwise_and(image, fusion_img, binary_img);
  cv::imshow("fusion_img", fusion_img);
  cv::waitKey(1);
  return binary_img;
}
std::vector<cv::Point> LaneFollow::getMaxContour(const cv::Mat & image)
{
  using std::vector;
  vector<vector<cv::Point>> contours;
  cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  auto max_contour = *std::max_element(
    contours.begin(), contours.end(),
    [&](const vector<cv::Point> & a, const vector<cv::Point> & b) {
      return cv::contourArea(a) > cv::contourArea(b);
    });
  //   int max_contour_idx = std::distance(contours.begin(), max_contour);
  return max_contour;
}

std::vector<std::vector<cv::Point>> LaneFollow::getContours(const cv::Mat & image)
{
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  vector<bool> remove_flag(contours.size());

  vector<vector<cv::Point>> contours_result;
  contours_result.reserve(contours.size());
  for (int i = 0; i < contours.size(); ++i) {
    if (remove_flag[i]) continue;
    auto & contour = contours[i];
    if (cv::contourArea(contour) > max_contour_area || contour.size() < 20) remove_flag[i] = true;
    contours_result.emplace_back(contour);
  }
}
}  // namespace lane_follow