cmake_minimum_required(VERSION 3.8)
project(ros_basic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install (PROGRAMS
  learning_ros/publisher.py
  learning_ros/subscriber.py
  learning_ros/move_robot.py
  learning_ros/move_robot_server.py
  learning_ros/move_robot_client.py
  learning_ros/simple_server.py
  tf/static.py
  tf/dynamic.py
  tf/safety.py
  tf/visual.py
  tf/aeb.py
  tf/visual_danger.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
