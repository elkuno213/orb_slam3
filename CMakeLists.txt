# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# ORB-SLAM3 #

cmake_minimum_required(VERSION 3.15...4.1)
project(orb_slam3 VERSION 1.1.0 LANGUAGES CXX)

include(GNUInstallDirs)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Dependencies #

find_package(OpenCV 4.4 REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2)
find_package(Boost COMPONENTS program_options serialization REQUIRED)
find_package(spdlog REQUIRED)
find_package(GTest REQUIRED)
find_package(DBoW2 REQUIRED)
find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenSSL REQUIRED)

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# orb_slam3 shared library #

add_library(${PROJECT_NAME} SHARED
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Atlas.cc
  src/Map.cc
  src/MapDrawer.cc
  src/Optimizer.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Viewer.cc
  src/ImuTypes.cc
  src/G2oTypes.cc
  src/CameraModels/Pinhole.cpp
  src/CameraModels/KannalaBrandt8.cpp
  src/OptimizableTypes.cpp
  src/MLPnPsolver.cpp
  src/GeometricTools.cc
  src/TwoViewReconstruction.cc
  src/Config.cc
  src/Settings.cc
  src/LoggingUtils.cc
  src/Common/Common.cc
  src/Common/EuRoC.cc
  src/Common/KITTI.cc
  src/Common/TUM.cc
  src/Common/TUMVI.cc
  src/Common/DatasetRunner.cc
  src/Common/DatasetRunners.cc
  include/System.h
  include/Tracking.h
  include/LocalMapping.h
  include/LoopClosing.h
  include/ORBextractor.h
  include/ORBmatcher.h
  include/FrameDrawer.h
  include/Converter.h
  include/MapPoint.h
  include/KeyFrame.h
  include/Atlas.h
  include/Map.h
  include/MapDrawer.h
  include/Optimizer.h
  include/Frame.h
  include/KeyFrameDatabase.h
  include/Sim3Solver.h
  include/Viewer.h
  include/ImuTypes.h
  include/G2oTypes.h
  include/CameraModels/GeometricCamera.h
  include/CameraModels/Pinhole.h
  include/CameraModels/KannalaBrandt8.h
  include/OptimizableTypes.h
  include/MLPnPsolver.h
  include/GeometricTools.h
  include/TwoViewReconstruction.h
  include/SerializationUtils.h
  include/Config.h
  include/Settings.h
  include/LoggingUtils.h
  include/Common/Common.h
  include/Common/EuRoC.h
  include/Common/KITTI.h
  include/Common/TUM.h
  include/Common/TUMVI.h
  include/Common/DatasetRunner.h
  include/Common/DatasetRunners.h
)

add_library(orb_slam3::orb_slam3 ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/CameraModels>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_include_directories(${PROJECT_NAME} SYSTEM
  PUBLIC
    ${Pangolin_INCLUDE_DIRS}
)

target_compile_options(${PROJECT_NAME}
  PRIVATE
    -Wall
    $<$<CONFIG:Release>:-O3>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${Pangolin_LIBRARIES}
    DBoW2::DBoW2
    g2o::g2o
    Sophus::Sophus
    Boost::serialization
    Boost::program_options
    spdlog::spdlog
  PRIVATE
    OpenSSL::Crypto
)

if(realsense2_FOUND)
  target_include_directories(${PROJECT_NAME}
    SYSTEM PUBLIC
      ${realsense_INCLUDE_DIR}
  )
  target_sources(${PROJECT_NAME} PRIVATE
    src/Common/RealSense.cc
    include/Common/RealSense.h
  )
  target_link_libraries(${PROJECT_NAME}
    PUBLIC
      ${realsense2_LIBRARY}
  )
endif()

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Install #

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
)

install(
  DIRECTORY include/
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
  FILES_MATCHING PATTERN "*.h"
)

install(
  EXPORT ${PROJECT_NAME}Targets
  NAMESPACE orb_slam3::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Unified offline binary #

add_executable(orb_slam3_offline src/orb_slam3_offline.cc)
target_link_libraries(orb_slam3_offline PRIVATE orb_slam3::orb_slam3)
install(TARGETS orb_slam3_offline DESTINATION "${CMAKE_INSTALL_BINDIR}")

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Unified RealSense binary #

if(realsense2_FOUND)
  add_executable(orb_slam3_realsense src/orb_slam3_realsense.cc)
  target_link_libraries(orb_slam3_realsense PRIVATE orb_slam3::orb_slam3)
  install(TARGETS orb_slam3_realsense DESTINATION "${CMAKE_INSTALL_BINDIR}")
endif()

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Tests #

include(CTest)
if(BUILD_TESTING)
  include(GoogleTest)

  set(TEST_FILES
    test/LoggingUtils_test.cc
    test/Settings_test.cc
    test/MapDrawer_test.cc
    test/Viewer_test.cc
    test/Tracking_test.cc
    test/DatasetRunner_test.cc
  )

  foreach(TEST_FILE ${TEST_FILES})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${TEST_NAME}
      PRIVATE
        orb_slam3::orb_slam3
        GTest::gtest
        GTest::gtest_main
    )
    gtest_discover_tests(${TEST_NAME})
  endforeach()
endif()

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Examples #

function(add_orb_slam3_example target_name source_file output_dir)
  add_executable(${target_name} ${source_file})
  target_link_libraries(${target_name} PRIVATE orb_slam3::orb_slam3)
  set_target_properties(${target_name} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/Examples/${output_dir}"
  )
  install(TARGETS ${target_name} DESTINATION "${CMAKE_INSTALL_BINDIR}")
endfunction()

option(BUILD_REALSENSE_EXAMPLES "Build legacy RealSense live-sensor example binaries" OFF)

if(BUILD_REALSENSE_EXAMPLES AND realsense2_FOUND)
  # RGB-D
  add_orb_slam3_example(rgbd_realsense_D435i Examples/RGB-D/rgbd_realsense_D435i.cc RGB-D)

  # RGB-D Inertial
  add_orb_slam3_example(
    rgbd_inertial_realsense_D435i
    Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc
    RGB-D-Inertial
  )

  # Stereo
  add_orb_slam3_example(stereo_realsense_t265  Examples/Stereo/stereo_realsense_t265.cc  Stereo)
  add_orb_slam3_example(stereo_realsense_D435i Examples/Stereo/stereo_realsense_D435i.cc Stereo)

  # Monocular
  add_orb_slam3_example(mono_realsense_t265  Examples/Monocular/mono_realsense_t265.cc  Monocular)
  add_orb_slam3_example(mono_realsense_D435i Examples/Monocular/mono_realsense_D435i.cc Monocular)

  # Monocular Inertial
  add_orb_slam3_example(
    mono_inertial_realsense_t265
    Examples/Monocular-Inertial/mono_inertial_realsense_t265.cc
    Monocular-Inertial
  )
  add_orb_slam3_example(
    mono_inertial_realsense_D435i
    Examples/Monocular-Inertial/mono_inertial_realsense_D435i.cc
    Monocular-Inertial
  )

  # Stereo Inertial
  add_orb_slam3_example(
    stereo_inertial_realsense_t265
    Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc
    Stereo-Inertial
  )
  add_orb_slam3_example(
    stereo_inertial_realsense_D435i
    Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc
    Stereo-Inertial
  )

  # Calibration
  add_orb_slam3_example(
    recorder_realsense_D435i
    Examples/Calibration/recorder_realsense_D435i.cc
    Calibration
  )
  add_orb_slam3_example(
    recorder_realsense_T265
    Examples/Calibration/recorder_realsense_T265.cc
    Calibration
  )
endif()
