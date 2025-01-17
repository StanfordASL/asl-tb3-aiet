cmake_minimum_required(VERSION 3.8)
project(asl_tb3_aiet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetMarker.msg"
)

install(PROGRAMS
    scripts/frontier_explorer.py
    scripts/heading_controller.py
    scripts/perception_controller.py
    scripts/navigator.py
    scripts/keyboard_teleop.py
    scripts/icp_node.py
    scripts/icp_node_scpd.py
    scripts/data_collector.py
    scripts/localize_targets.py
    scripts/task_execution.py
    scripts/e2e.py
    scripts/simple_nav.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz rqt
        DESTINATION share/${PROJECT_NAME})

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

ament_package()
