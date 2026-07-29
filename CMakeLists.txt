cmake_minimum_required(VERSION 3.8)
project(ndt_localization)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ Standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)

# Thirdparty libraries
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree registration)

set(dependencies
  rclcpp
  builtin_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# Localization state, validation, workload-control, and initialization core
add_library(localization_core src/localization_core.cpp)
target_link_libraries(localization_core Eigen3::Eigen)
target_include_directories(localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(point_cloud_utils src/point_cloud_utils.cpp)
target_link_libraries(
  point_cloud_utils
  localization_core
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
target_include_directories(point_cloud_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_library(robust_initializer src/robust_initializer.cpp)
target_link_libraries(
  robust_initializer
  localization_core
  point_cloud_utils
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
target_include_directories(robust_initializer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Localization node
add_executable(localization_node src/localization.cpp)
target_link_libraries(
  localization_node
  localization_core
  point_cloud_utils
  robust_initializer
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(localization_node ${dependencies})

# Install targets
install(
  TARGETS
    localization_node
    localization_core
    point_cloud_utils
    robust_initializer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install production configuration
install(
  FILES config/localization.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    localization_core_test
    test/test_localization_core.cpp
  )
  if(TARGET localization_core_test)
    target_link_libraries(
      localization_core_test
      localization_core
      Eigen3::Eigen
    )
  endif()
  ament_add_gtest(
    point_cloud_utils_test
    test/test_point_cloud_utils.cpp
  )
  if(TARGET point_cloud_utils_test)
    target_link_libraries(
      point_cloud_utils_test
      point_cloud_utils
      ${PCL_LIBRARIES}
      Eigen3::Eigen
    )
  endif()
endif()

ament_package()
