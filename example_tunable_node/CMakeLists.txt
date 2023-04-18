cmake_minimum_required(VERSION 3.8)
project(example_tunable)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(example_parameters src/parameters.yaml)

add_executable(example_tunable src/example_tunable.cpp)
target_compile_features(example_tunable PUBLIC cxx_std_17)
target_link_libraries(example_tunable example_parameters rclcpp::rclcpp)

install(TARGETS example_tunable
  DESTINATION lib/example_tunable)

ament_package()
