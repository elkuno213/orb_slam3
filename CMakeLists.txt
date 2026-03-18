# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# ORB-SLAM3 #

cmake_minimum_required(VERSION 3.15...4.1)
project(orb_slam3 VERSION 2.0.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Build options                                                                                    #

option(ENABLE_ASAN "Enable AddressSanitizer + UndefinedBehaviorSanitizer" OFF)
option(ENABLE_TSAN "Enable ThreadSanitizer" OFF)

if(ENABLE_ASAN AND ENABLE_TSAN)
  message(FATAL_ERROR "ENABLE_ASAN and ENABLE_TSAN are mutually exclusive")
endif()

if(ENABLE_ASAN)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

if(ENABLE_TSAN)
  add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
  add_link_options(-fsanitize=thread)
endif()

option(ENABLE_CLANG_TIDY "Enable clang-tidy static analysis" OFF)
option(CLANG_TIDY_WARNINGS_AS_ERRORS "Treat clang-tidy diagnostics as errors" OFF)

if(ENABLE_CLANG_TIDY)
  find_program(CLANG_TIDY_EXE NAMES clang-tidy REQUIRED)
  set(_clang_tidy_cmd "${CLANG_TIDY_EXE}")
  if(CLANG_TIDY_WARNINGS_AS_ERRORS)
    list(APPEND _clang_tidy_cmd "--warnings-as-errors=*")
  endif()
  set(CMAKE_CXX_CLANG_TIDY "${_clang_tidy_cmd}")
  message(STATUS "clang-tidy enabled: ${CLANG_TIDY_EXE}"
                 " (warnings-as-errors: ${CLANG_TIDY_WARNINGS_AS_ERRORS})")
endif()

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Dependencies #

find_package(OpenCV 4.4 REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
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
  src/Common/DatasetRunner.cc
  src/Common/EuRoCRunner.cc
  src/Common/TumRunner.cc
  src/Common/TumViRunner.cc
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
  include/Common/DatasetRunner.h
  include/Common/EuRoCRunner.h
  include/Common/TumRunner.h
  include/Common/TumViRunner.h
)

add_library(orb_slam3::orb_slam3 ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_EXTENSIONS OFF)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/CameraModels>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_include_directories(${PROJECT_NAME} SYSTEM
  PUBLIC
    ${Pangolin_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Thirdparty/g2o>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2>
)

target_compile_options(${PROJECT_NAME}
  PRIVATE
    -Wall
    -Werror=reorder
    -Werror=sign-compare
    -Werror=unused-variable
    -Werror=unused-but-set-variable
    -Werror=maybe-uninitialized
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

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}"
)

# ──────────────────────────────────────────────────────────────────────────────────────────────── #
# Unified offline binary #

add_executable(orb_slam3_offline src/orb_slam3_offline.cc)
target_link_libraries(orb_slam3_offline PRIVATE orb_slam3::orb_slam3)
install(TARGETS orb_slam3_offline DESTINATION "${CMAKE_INSTALL_BINDIR}")

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
