cmake_minimum_required(VERSION 3.8)
project(beefast_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(bondcpp REQUIRED)
find_package(bond REQUIRED)
find_package(action_msgs REQUIRED)

set(dependencies
    rclcpp
    lifecycle_msgs
    rclcpp_action
    rclcpp_lifecycle
    bondcpp
    bond
    action_msgs
)

include_directories(include)

set(library_name ${PROJECT_NAME}_core)
#add_subdirectory(src)

add_library(${library_name} SHARED
  src/node_utils.cpp
  src/lifecycle_service_client.cpp
  src/node_thread.cpp
  src/lifecycle_node.cpp
  src/string_utils.cpp
  # src/node_parameter.cpp
)

ament_target_dependencies(${library_name}
  rclcpp
  lifecycle_msgs
  rclcpp_lifecycle
  bondcpp
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
