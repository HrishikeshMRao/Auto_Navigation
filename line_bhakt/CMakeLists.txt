cmake_minimum_required(VERSION 3.8)
project(line_bhakt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)
set(SOURCES1 /home/fiend/LineFollowingRobot/ros_ws/src/line_bhakt/src/ImageCapture.cpp)
add_executable(ImageCapture ${SOURCES1})
ament_target_dependencies(ImageCapture rclcpp cv_bridge OpenCV std_msgs sensor_msgs image_transport geometry_msgs)
target_link_libraries(ImageCapture ${OpenCV_LIBS})

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

install(
  DIRECTORY include launch worlds src
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    ImageCapture 
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
