#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "example_parameters.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("example_tunable");
  auto param_listener = std::make_shared<example::ParamListener>(node);
  auto params = param_listener->get_params();

  while(1) {
    if (param_listener->is_old(params)) {
      params = param_listener->get_params();

      RCLCPP_INFO(node->get_logger(), "red: %ld", params.red);
      RCLCPP_INFO(node->get_logger(), "alpha: %f", params.alpha);
    }

    rclcpp::sleep_for(std::chrono::seconds(1));
  }
}
