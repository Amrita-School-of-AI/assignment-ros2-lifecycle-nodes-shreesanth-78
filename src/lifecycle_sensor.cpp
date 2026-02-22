#include <memory>
#include <chrono>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class LifecycleSensor : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleSensor()
  : LifecycleNode("lifecycle_sensor")
  {
    RCLCPP_INFO(get_logger(), "Lifecycle Sensor node created");
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float32>(
      "sensor_data", 10);

    RCLCPP_INFO(get_logger(), "Sensor configured");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    publisher_->on_activate();

    timer_ = this->create_wall_timer(
      1s, std::bind(&LifecycleSensor::publish_data, this));

    RCLCPP_INFO(get_logger(), "Sensor activated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    publisher_->on_deactivate();
    timer_.reset();

    RCLCPP_INFO(get_logger(), "Sensor deactivated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    publisher_.reset();

    RCLCPP_INFO(get_logger(), "Sensor cleaned up");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void publish_data()
  {
    auto message = std_msgs::msg::Float32();
    message.data = static_cast<float>(rand() % 100);

    publisher_->publish(message);

    RCLCPP_INFO(get_logger(), "Publishing sensor data: %.2f", message.data);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LifecycleSensor>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
