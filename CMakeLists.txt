cmake_minimum_required(VERSION 3.6)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(
  mc_moveit
  LANGUAGES C CXX
  VERSION 1.0.0
)

include(GNUInstallDirs)

find_package(mc_rtc REQUIRED)

if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin must be installed to build mc_moveit")
endif()

find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(octomap_msgs REQUIRED)

set(DEFAULT_PLANNER_CONFIG_DIR ${CMAKE_INSTALL_FULL_DATADIR}/mc_moveit/)
install(FILES etc/default_planner_config.yml DESTINATION ${DEFAULT_PLANNER_CONFIG_DIR})

configure_file(
  include/mc_moveit/config.in.h ${PROJECT_BINARY_DIR}/include/mc_moveit/config.h
)

set(mc_moveit_HDR
    include/mc_moveit/api.h include/mc_moveit/BSplineTrajectoryTask.h
    include/mc_moveit/Planner.h include/mc_moveit/PostureTrajectoryTask.h
    ${PROJECT_BINARY_DIR}/include/mc_moveit/config.h
)

set(mc_moveit_SRC src/BSplineTrajectoryTask.cpp src/Planner.cpp
                  src/PostureTrajectoryTask.cpp
)

add_library(mc_moveit SHARED ${mc_moveit_SRC} ${mc_moveit_HDR})
target_include_directories(
  mc_moveit
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include> $<INSTALL_INTERFACE:include>
)
target_link_libraries(mc_moveit PUBLIC mc_rtc::mc_control mc_rtc::mc_rtc_ros)
target_link_libraries(
  mc_moveit
  PUBLIC moveit_ros_planning_interface::moveit_common_planning_interface_objects
         octomap_msgs::octomap_msgs__rosidl_typesupport_cpp
)
target_compile_definitions(mc_moveit PRIVATE -DMC_MOVEIT_EXPORTS)
set_target_properties(
  mc_moveit PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION}
)
add_library(mc_moveit::mc_moveit ALIAS mc_moveit)

install(FILES ${mc_moveit_HDR} DESTINATION include/mc_moveit)
install(
  TARGETS mc_moveit
  EXPORT mc_moveitTargets
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)

find_package(Boost REQUIRED COMPONENTS program_options)
add_executable(mc_moveit_sample src/sample.cpp)
target_link_libraries(
  mc_moveit_sample PUBLIC mc_moveit mc_rtc::mc_control Boost::program_options
)

add_subdirectory(cmake)
