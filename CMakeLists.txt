cmake_minimum_required(VERSION 3.8)
project(pololu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# maestro
add_executable(maestro src/maestroNode.cpp src/maestroDevice.cpp src/usbDevice.cpp)
target_include_directories(maestro PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(maestro PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(maestro -lusb-1.0)
ament_target_dependencies(maestro rclcpp std_msgs)

# simple motor controller (SMC)
add_executable(smc src/smcNode.cpp src/smcDevice.cpp src/usbDevice.cpp)
target_include_directories(smc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(smc PUBLIC c_std_99 cxx_std_17)
target_link_libraries(smc -lusb-1.0)
ament_target_dependencies(smc rclcpp std_msgs)

# jrk
add_executable(jrk src/jrkNode.cpp src/jrkDevice.cpp)
target_include_directories(jrk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(jrk PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(jrk rclcpp std_msgs)

# install targets
install(TARGETS maestro smc jrk
  DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
