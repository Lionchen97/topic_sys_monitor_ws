cmake_minimum_required(VERSION 3.8)
project(status_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(status_interfaces REQUIRED)
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(LIBSTATGRAB REQUIRED IMPORTED_TARGET libstatgrab)

add_executable(sys_status_publisher src/sys_status_publisher.cpp)
ament_target_dependencies(sys_status_publisher rclcpp status_interfaces)
#target_include_directories(sys_status_publisher PRIVATE
#  ${LIBSTATGRAB_INCLUDE_DIRS}
#)
#target_link_libraries(sys_status_publisher
#  ${LIBSTATGRAB_LIBRARIES}
#)


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

install(TARGETS sys_status_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
