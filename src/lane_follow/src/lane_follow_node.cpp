// 1.包含头文件；
#include "lane_follow/lane_follow.hpp"

#include <rclcpp/rclcpp.hpp>
// #include "lane_follow/liblane_follow.so"
// #include "lane_follow/lane_follow_node"
#include <image_transport/image_transport.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

// 3.自定义节点类；
namespace lane_follow
{
class LaneFollowNode : public rclcpp::Node
{
public:
  explicit LaneFollowNode() : Node("lane_follow_node_cpp")
  {
    RCLCPP_INFO(this->get_logger(), "LaneFollow Node Start!");
    init_param();
  }
  void init_param()
  {
    using namespace std::chrono_literals;
    // auto declare_param = [this](
    //                        std::string name, auto default_value, std::string description = "") {
    //   auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    //   param_desc.description = description;
    //   return this->declare_parameter(name, default_value, param_desc);
    // };
    // auto upper_boundary = declare_param("upper_boundary", , "ss");
    // auto lower_boundary = declare_param("lower_boundary", , "ss");
    auto upper_boundary = this->declare_parameter<std::vector<int64>>("upper_boundary");
    auto lower_boundary = this->declare_parameter<std::vector<int64>>("lower_boundary");
    auto max_contour_area = this->declare_parameter<double>("max_contour_area");
    lane_follow_ =
      std::make_shared<lane_follow::LaneFollow>(lower_boundary, upper_boundary, max_contour_area);
    timer_ =
      rclcpp::create_timer(this, get_clock(), 10ms, std::bind(&LaneFollowNode::onConnect, this));
    img_pub_ = image_transport::create_publisher(this, "/det_lane");
  }
  void onConnect()
  {
    using std::placeholders::_1;
    if (img_pub_.getNumSubscribers() == 0) {
      img_pub_.shutdown();
    } else if (!img_sub_) {
      img_sub_ = image_transport::create_subscription(
        this, "/raw_image", std::bind(&LaneFollowNode::onImage, this, std::placeholders::_1), "raw",
        rmw_qos_profile_sensor_data);
    }
  }
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv_bridge::CvImagePtr in_img_ptr;
    try {
      in_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    auto binary_img = lane_follow_->preProcess({in_img_ptr->image});
    auto contour = lane_follow_->getMaxContour(binary_img);
    cv::drawContours(in_img_ptr->image, contour, -1, cv::Scalar(0, 255, 0));
    img_pub_.publish(in_img_ptr->toImageMsg());
  }

private:
  std::shared_ptr<lane_follow::LaneFollow> lane_follow_;
  image_transport::Subscriber img_sub_;
  image_transport::Publisher img_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace lane_follow
int main(int argc, char const * argv[])
{
  // 2.初始化ROS2客户端；
  rclcpp::init(argc, argv);
  // 4.调用spain函数，并传入节点对象指针；
  rclcpp::spin(std::make_shared<lane_follow::LaneFollowNode>());
  // 5.资源释放
  rclcpp::shutdown();
  return 0;
}