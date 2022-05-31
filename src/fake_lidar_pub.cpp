#include <chrono>
#include <functional>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

template <class T>
bool parse_param(const std::string &param_name, T &param_dest, rclcpp::Node& node)
{
  node.declare_parameter<T>(param_name); 
  if (!node.get_parameter(param_name, param_dest))
  {
    RCLCPP_ERROR(node.get_logger(), "Could not load param '%s'", param_name.c_str());
    return false;
  }
  else
  {
    RCLCPP_INFO_STREAM(node.get_logger(), "Loaded '" << param_name << "' = '" << param_dest << "'");
  }
  return true;
}

class FakeLidarPublisher : public rclcpp::Node
{
public:
  FakeLidarPublisher()
  : Node("fake_lidar_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("topic", 10);

    RCLCPP_INFO(get_logger(), "-------------- Loading parameters --------------");
    bool loaded_successfully = true;
    std::string frame_id;
    double rate;
    double blocking_distance;
    loaded_successfully &= parse_param("rate", rate, *this);
    loaded_successfully &= parse_param("blocking_distance", blocking_distance, *this);
    loaded_successfully &= parse_param("frame_id", frame_id, *this);

    // Check if all parameters were loaded correctly
    if (!loaded_successfully) {
      const std::string str = "Could not load all non-optional parameters. Shutting down.";
      RCLCPP_ERROR(get_logger(), "%s", str.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(), "-------------- Parameters loaded --------------");

    message_ = sensor_msgs::msg::LaserScan();
    message_.header.frame_id = frame_id;
    message_.angle_min = -3.1241400241851807;
    message_.angle_max = 3.1241400241851807;
    message_.angle_increment = 0.008690236136317253;
    message_.time_increment = 0.0;
    message_.scan_time = 0.0;
    message_.range_min = 0.15000000596046448;
    message_.range_max = 14.0;

    const int number_of_elements = int((message_.angle_max - message_.angle_min) / message_.angle_increment);

    for (int it = 0; it < number_of_elements ; it++) {
      message_.ranges.push_back(blocking_distance);
    }

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate), std::bind(&FakeLidarPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "Publishing data !!!");

    message_.header.stamp = this->get_clock()->now();

    publisher_->publish(message_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  sensor_msgs::msg::LaserScan message_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeLidarPublisher>());
  rclcpp::shutdown();
  return 0;
}
