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

## Master include
add_library(adynamics INTERFACE)
ament_export_targets(export_adynamics HAS_LIBRARY_TARGET)

install(
    TARGETS adynamics
    EXPORT export_adynamics
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
ament_export_include_directories(include)

## utils
add_library(utils src/utils.cpp)
target_link_libraries(utils Eigen3::Eigen jsoncpp)
ament_export_targets(utils HAS_LIBRARY_TARGET)

install(
  TARGETS utils
  EXPORT utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

## rigid_body_dynamics
add_library(rigid_body_dynamics src/rigid_body_dynamics.cpp)
target_link_libraries(rigid_body_dynamics utils Eigen3::Eigen jsoncpp)
ament_export_targets(rigid_body_dynamics HAS_LIBRARY_TARGET)

install(
  TARGETS rigid_body_dynamics
  EXPORT rigid_body_dynamics
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

############################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

