cmake_minimum_required(VERSION 4.4.0)
project(direct_visual_lidar_calibration VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_VLCAL_ALIGN "Build viewer-free alignment library" ON)
option(BUILD_VLCAL_PREPROCESS "Build point cloud preprocessing library" ON)
option(BUILD_WITH_VIEWER "Build Iridescence viewer tools and executables" OFF)
option(BUILD_VLCAL_TESTS "Build unit tests" ON)
option(BUILD_WITH_MARCH_NATIVE "Enable -march=native" ON)
option(ENABLE_ROS "Build ROS preprocess tools" OFF)

if(BUILD_WITH_MARCH_NATIVE)
  add_compile_options(-march=native -mtune=native)
endif()

find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(VLCAL_INCLUDE_DIRS
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty/json/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty/Bonxai/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty/nanoflann/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/thirdparty/Sophus>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/direct_visual_lidar_calibration>
)

# ---------------------------------------------------------------------------
# vlcal_align — NID calibration without GTSAM / PCL / Iridescence
# ---------------------------------------------------------------------------
if(BUILD_VLCAL_ALIGN)
  find_package(Ceres REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc)
  find_package(Eigen3 REQUIRED)
  find_package(Boost REQUIRED COMPONENTS filesystem)

  add_library(vlcal_align SHARED
    src/camera/create_camera.cpp
    src/vlcal/common/frame.cpp
    src/vlcal/common/frame_cpu.cpp
    src/vlcal/common/estimate_fov.cpp
    src/vlcal/common/estimate_pose.cpp
    src/vlcal/common/scan_time.cpp
    src/vlcal/common/time_keeper.cpp
    src/vlcal/calib/cost_calculator_nid.cpp
    src/vlcal/calib/view_culling.cpp
    src/vlcal/calib/visual_camera_calibration.cpp
  )
  target_include_directories(vlcal_align PUBLIC ${VLCAL_INCLUDE_DIRS})
  target_link_libraries(vlcal_align PUBLIC
    Eigen3::Eigen
    opencv_core
    opencv_imgproc
    Boost::filesystem
    PRIVATE Ceres::ceres)
  if(OPENMP_FOUND)
    target_link_libraries(vlcal_align PUBLIC OpenMP::OpenMP_CXX)
  endif()
  add_library(vlcal::align ALIAS vlcal_align)
endif()

# ---------------------------------------------------------------------------
# vlcal_preprocess — integrators, CT factors, scan-time utilities
# ---------------------------------------------------------------------------
if(BUILD_VLCAL_PREPROCESS)
  find_package(PCL REQUIRED COMPONENTS io surface sample_consensus filters)
  find_package(GTSAM REQUIRED)
  find_package(Boost REQUIRED COMPONENTS filesystem program_options date_time)
  find_package(fmt REQUIRED)

  add_library(vlcal_preprocess SHARED
    src/vlcal/common/ivox.cpp
    src/vlcal/common/cloud_covariance_estimation.cpp
    src/vlcal/preprocess/estimate_lidar_fov.cpp
    src/vlcal/preprocess/generate_lidar_image.cpp
    src/vlcal/preprocess/static_point_cloud_integrator.cpp
    src/vlcal/preprocess/dynamic_point_cloud_integrator.cpp
  )
  if(TARGET gtsam)
    set(VLCAL_GTSAM_LIBS gtsam)
  else()
    set(VLCAL_GTSAM_LIBS ${GTSAM_LIBRARIES})
  endif()

  target_include_directories(vlcal_preprocess PUBLIC
    ${VLCAL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
  )
  if(GTSAM_INCLUDE_DIRS)
    target_include_directories(vlcal_preprocess PUBLIC ${GTSAM_INCLUDE_DIRS})
  endif()
  target_link_libraries(vlcal_preprocess PUBLIC
    ${Boost_LIBRARIES}
    ${VLCAL_GTSAM_LIBS}
    ${PCL_LIBRARIES}
    fmt::fmt
  )
  if(BUILD_VLCAL_ALIGN)
    target_link_libraries(vlcal_preprocess PUBLIC vlcal_align)
  endif()
  if(BUILD_WITH_VIEWER)
    find_package(Iridescence REQUIRED)
    target_sources(vlcal_preprocess PRIVATE src/vlcal/preprocess/preprocess.cpp)
    target_compile_definitions(vlcal_preprocess PRIVATE VLCAL_WITH_IRIDESCENCE)
    target_link_libraries(vlcal_preprocess PRIVATE Iridescence::Iridescence)
  endif()
  add_library(vlcal::preprocess ALIAS vlcal_preprocess)
endif()

# ---------------------------------------------------------------------------
# vlcal_viewer — Iridescence tools (optional)
# ---------------------------------------------------------------------------
if(BUILD_WITH_VIEWER)
  find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
  find_package(Iridescence REQUIRED)
  if(NOT TARGET vlcal_preprocess)
    message(FATAL_ERROR "BUILD_WITH_VIEWER requires BUILD_VLCAL_PREPROCESS")
  endif()

  add_library(vlcal_viewer SHARED
    src/vlcal/common/points_color_updater.cpp
    src/vlcal/common/visual_lidar_data_loader.cpp
    src/vlcal/common/visual_lidar_visualizer.cpp
  )
  target_include_directories(vlcal_viewer PUBLIC ${VLCAL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(vlcal_viewer PUBLIC vlcal_preprocess Iridescence::Iridescence ${OpenCV_LIBS})

  add_executable(preprocess_map src/preprocess_map.cpp)
  add_executable(initial_guess_manual src/initial_guess_manual.cpp)
  add_executable(initial_guess_auto src/initial_guess_auto.cpp)
  add_executable(calibrate src/calibrate.cpp)
  add_executable(viewer src/viewer.cpp)
  foreach(exe preprocess_map initial_guess_manual initial_guess_auto calibrate viewer)
    target_link_libraries(${exe} PRIVATE vlcal_viewer)
  endforeach()
endif()

# Legacy combined target for downstream tools that expect one library
if(BUILD_VLCAL_ALIGN AND BUILD_VLCAL_PREPROCESS)
  add_library(direct_visual_lidar_calibration INTERFACE)
  target_link_libraries(direct_visual_lidar_calibration INTERFACE vlcal_preprocess)
  if(BUILD_WITH_VIEWER)
    target_link_libraries(direct_visual_lidar_calibration INTERFACE vlcal_viewer)
  endif()
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_VLCAL_TESTS AND BUILD_VLCAL_ALIGN)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(vlcal_align_test tests/test_visual_calibration.cpp)
  target_link_libraries(vlcal_align_test PRIVATE vlcal_align GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(vlcal_align_test DISCOVERY_MODE PRE_TEST)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
if(BUILD_VLCAL_ALIGN)
  install(TARGETS vlcal_align EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
if(BUILD_VLCAL_PREPROCESS)
  install(TARGETS vlcal_preprocess EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/direct_visual_lidar_calibration)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/thirdparty/Sophus/sophus/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/direct_visual_lidar_calibration/sophus)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/thirdparty/json/include/nlohmann/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/direct_visual_lidar_calibration/nlohmann)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/thirdparty/nanoflann/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/direct_visual_lidar_calibration)

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${INSTALL_CONFIGDIR})

include(CMakePackageConfigHelpers)

set(VLCAL_PREPROCESS_DEPENDENCIES "")
if(BUILD_VLCAL_PREPROCESS)
  set(VLCAL_PREPROCESS_DEPENDENCIES [[
find_dependency(PCL REQUIRED)
find_dependency(GTSAM REQUIRED)
find_dependency(Boost REQUIRED COMPONENTS program_options date_time)
find_dependency(fmt REQUIRED)
]])
endif()

set(VLCAL_VIEWER_DEPENDENCIES "")
if(BUILD_WITH_VIEWER)
  set(VLCAL_VIEWER_DEPENDENCIES [[
find_dependency(Iridescence REQUIRED)
]])
endif()

write_basic_package_version_file(${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY AnyNewerVersion)
configure_package_config_file(${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR})
install(FILES
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${INSTALL_CONFIGDIR})
