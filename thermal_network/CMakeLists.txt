cmake_minimum_required(VERSION 3.8)
project(thermal_network)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(rosidl_default_generators REQUIRED)
include_directories(${BOOST_INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ThermalData.msg"
  LIBRARY_NAME ${PROJECT_NAME}
  DEPENDENCIES
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(thermal_data src/thermal_data.cpp)
target_link_libraries(thermal_data ${cpp_typesupport_target} ${BOOST_LIBRARIES})
ament_target_dependencies(thermal_data rclcpp std_msgs sensor_msgs)

install(TARGETS
  thermal_data
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  src
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
