cmake_minimum_required(VERSION 3.14)
project(slam_map_node CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# ---------------------------------------------------------------------------
# dimos-lcm (LCM message headers — fetched from GitHub, or provided by nix)
# ---------------------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG        main
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

# ---------------------------------------------------------------------------
# nanoflann (header-only KD-tree for Scan Context ring-key retrieval)
# ---------------------------------------------------------------------------
FetchContent_Declare(nanoflann
  GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
  GIT_TAG        v1.6.2
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(nanoflann)

# ---------------------------------------------------------------------------
# LCM runtime
# ---------------------------------------------------------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

# ---------------------------------------------------------------------------
# OctoMap
# ---------------------------------------------------------------------------
find_package(octomap REQUIRED)

# ---------------------------------------------------------------------------
# Eigen3
# ---------------------------------------------------------------------------
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ---------------------------------------------------------------------------
# Boost (required by GTSAM's cmake exports)
# ---------------------------------------------------------------------------
find_package(Boost REQUIRED)

# ---------------------------------------------------------------------------
# GTSAM
# ---------------------------------------------------------------------------
find_package(GTSAM REQUIRED)

# ---------------------------------------------------------------------------
# Common headers (shared with dynamic_map — symlink or nix override)
# ---------------------------------------------------------------------------
if(DEFINED SLAM_MAP_COMMON_DIR)
  set(COMMON_DIR "${SLAM_MAP_COMMON_DIR}")
else()
  set(COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/common")
endif()

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
add_executable(slam_map_node
  slam_map_node.cpp
  icp_odometry.cpp
  scan_context.cpp
  pose_graph.cpp
  dynamic_mapper.cpp
)

# Create a "common" symlink in the build dir pointing to COMMON_DIR
# so that #include "common/foo.hpp" resolves regardless of nix or local build
file(CREATE_LINK ${COMMON_DIR} ${CMAKE_CURRENT_BINARY_DIR}/common SYMBOLIC)

target_include_directories(slam_map_node PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}  # resolves "common/foo.hpp" via the symlink
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${nanoflann_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(slam_map_node PRIVATE
  ${LCM_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  Eigen3::Eigen
  gtsam
)

target_link_directories(slam_map_node PRIVATE
  ${LCM_LIBRARY_DIRS}
)

install(TARGETS slam_map_node DESTINATION bin)
