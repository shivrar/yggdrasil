cmake_minimum_required(VERSION 3.8)
project(yggdrasil)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

add_library(${PROJECT_NAME}  STATIC
        include/yggdrasil/bt_nodes.cpp)
target_include_directories(${PROJECT_NAME}  PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_definitions(${PROJECT_NAME} 
        PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(${PROJECT_NAME} 
        "action_tutorials_interfaces"
        "rclcpp"
        "rclcpp_action"
        "std_srvs"
        "rclcpp_components")
install(TARGETS
        ${PROJECT_NAME} 
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)


ament_target_dependencies(${PROJECT_NAME}  behaviortree_cpp behaviortree_ros2)

add_library(bt_sample_nodes STATIC
        sample_nodes/dummy_nodes.cpp
        sample_nodes/movebase_node.cpp )
target_include_directories(bt_sample_nodes PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_definitions(bt_sample_nodes
        PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(bt_sample_nodes
        "action_tutorials_interfaces"
        "rclcpp"
        "rclcpp_action"
        "rclcpp_components")
rclcpp_components_register_node(bt_sample_nodes PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_bt_sample_nodes)
install(TARGETS
        bt_sample_nodes
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)


ament_target_dependencies(bt_sample_nodes behaviortree_cpp behaviortree_ros2)

add_executable(sample_tree src/sample_tree.cpp)
target_include_directories(sample_tree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
        sample_nodes
)
target_link_libraries(sample_tree bt_sample_nodes)
target_compile_features(sample_tree PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  sample_tree
  "rclcpp"
  "action_tutorials_interfaces"
  "rclcpp_lifecycle"
  "std_msgs"
  "std_srvs"
  "behaviortree_cpp"
  "behaviortree_ros2"
)

install(TARGETS sample_tree
  DESTINATION lib/${PROJECT_NAME})

add_executable(main_tree src/main.cpp)
target_include_directories(main_tree PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        include/yggdrasil
        sample_nodes
        ${PROJECT_NAME}
)
target_link_libraries(main_tree ${PROJECT_NAME} bt_sample_nodes)
target_compile_features(main_tree PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
        main_tree
        "rclcpp"
        "action_tutorials_interfaces"
        "rclcpp_lifecycle"
        "std_msgs"
        "behaviortree_cpp"
        "behaviortree_ros2"
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
