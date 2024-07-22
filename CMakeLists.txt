cmake_minimum_required(VERSION 3.8)
project(yggdrasil)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_library(bt_sample_nodes STATIC
        sample_nodes/dummy_nodes.cpp
        sample_nodes/movebase_node.cpp )

ament_target_dependencies(bt_sample_nodes behaviortree_cpp)

add_executable(main_tree src/main_tree.cpp)
target_include_directories(main_tree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
        sample_nodes
)
target_link_libraries(main_tree bt_sample_nodes)
target_compile_features(main_tree PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  main_tree
  "rclcpp"
  "rclcpp_lifecycle"
  "std_msgs"
  "behaviortree_cpp"
)

install(TARGETS main_tree
  DESTINATION lib/${PROJECT_NAME})

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
