cmake_minimum_required(VERSION 3.8)
project(aaveq_dynamics)

add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 20)


############################################################################

# Dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

############################################################################

# Libraries

include_directories(include)

install(
  DIRECTORY include/aaveq_dynamics
  DESTINATION include)

############################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

