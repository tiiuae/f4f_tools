#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_gps_position.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

class PoseDataRepublisher : public rclcpp::Node
{
public:
  PoseDataRepublisher()
  : Node("pose_data_republisher")
  {
    publisher_gps_position_ = this->create_publisher<px4_msgs::msg::VehicleGpsPosition>("gps_topic_out", 10);
    publisher_local_position_ = this->create_publisher<px4_msgs::msg::VehicleLocalPosition>("local_topic_out", 10);

    subscription_gps_position_ = this->create_subscription<px4_msgs::msg::VehicleGpsPosition>("gps_topic_in", 10, std::bind(&PoseDataRepublisher::gps_position_callback, this, std::placeholders::_1));
    subscription_local_position_ = this->create_subscription<px4_msgs::msg::VehicleGpsPosition>("local_topic_in", 10, std::bind(&PoseDataRepublisher::local_position_callback, this, std::placeholders::_1));

  }

private:
  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "Publishing local data !!!");

    publisher_local_position_->publish(*msg);
  }

  void gps_position_callback(const px4_msgs::msg::VehicleGpsPosition::SharedPtr msg) const
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "Publishing gps data !!!");

    publisher_gps_position_->publish(*msg);
  }

  rclcpp::Publisher<px4_msgs::msg::VehicleGpsPosition>::SharedPtr publisher_gps_position_;
  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr publisher_local_position_;

  rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr subscription_gps_position_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_local_position_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseDataRepublisher>());
  rclcpp::shutdown();
  return 0;
}
