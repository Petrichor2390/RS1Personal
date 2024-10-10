cmake_minimum_required(VERSION 3.8)
project(sprint3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlebot3_navigation2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(objectExcludeExe src/objectExclude.cpp)
ament_target_dependencies(objectExcludeExe rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)
target_link_libraries(objectExcludeExe yaml-cpp ${OpenCV_LIBS})

add_executable(mapOverlayExe src/mapOverlay.cpp)
ament_target_dependencies(mapOverlayExe rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)
target_link_libraries(mapOverlayExe yaml-cpp ${OpenCV_LIBS})

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

install(TARGETS
  objectExcludeExe
  mapOverlayExe
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
