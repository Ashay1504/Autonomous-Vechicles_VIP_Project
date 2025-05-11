#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class RacecarController : public rclcpp::Node {
public:
  RacecarController() : Node("racecar_controller") {
    // Declare parameters (with defaults)
    this->declare_parameter<double>("max_speed", 0.5); // m/s
    this->declare_parameter<double>("slow_down_distance", 0.6096); // 2ft in meters
    this->declare_parameter<double>("stop_distance", 0.3048); // 1ft in meters

    // Get parameters
    max_speed_ = this->get_parameter("max_speed").as_double();
    slow_down_distance_ = this->get_parameter("slow_down_distance").as_double();
    stop_distance_ = this->get_parameter("stop_distance").as_double();

    // Subscriber and Publisher
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&RacecarController::scanCallback, this, _1)
    );
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    float min_distance = std::numeric_limits<float>::infinity();
    float angle_min = scan_msg->angle_min;
    float angle_increment = scan_msg->angle_increment;

    // Check scans within ±30 degrees (front of the car)
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
      float angle = angle_min + i * angle_increment;
      if (angle >= -M_PI/6 && angle <= M_PI/6) { // -30° to +30°
        float distance = scan_msg->ranges[i];
        // Validate distance reading
        if (distance >= scan_msg->range_min && distance <= scan_msg->range_max) {
          if (distance < min_distance) {
            min_distance = distance;
          }
        }
      }
    }

    // Control logic
    auto cmd_vel = geometry_msgs::msg::Twist();
    if (std::isinf(min_distance)) {
      RCLCPP_WARN(this->get_logger(), "No valid obstacle detected. Stopping.");
      cmd_vel.linear.x = 0.0;
    } else if (min_distance <= stop_distance_) {
      cmd_vel.linear.x = 0.0; // Stop
    } else if (min_distance <= slow_down_distance_) {
      // Proportional speed reduction between 2ft and 1ft
      cmd_vel.linear.x = max_speed_ * ((min_distance - stop_distance_) / (slow_down_distance_ - stop_distance_));
    } else {
      cmd_vel.linear.x = max_speed_; // Full speed
    }

    cmd_vel.angular.z = 0.0; // No steering
    cmd_vel_pub_->publish(cmd_vel);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  double max_speed_, slow_down_distance_, stop_distance_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacecarController>());
  rclcpp::shutdown();
  return 0;
}
