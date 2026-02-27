cmake_minimum_required(VERSION 3.12)
project(DBoW2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3")

set(HDRS_DBOW2
    DBoW2/BowVector.h
    DBoW2/FORB.h
    DBoW2/FClass.h
    DBoW2/FeatureVector.h
    DBoW2/ScoringObject.h
    DBoW2/TemplatedVocabulary.h
)
set(SRCS_DBOW2 DBoW2/BowVector.cpp DBoW2/FORB.cpp DBoW2/FeatureVector.cpp DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS DUtils/Random.h DUtils/Timestamp.h)
set(SRCS_DUTILS DUtils/Random.cpp DUtils/Timestamp.cpp)

find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3.0 QUIET)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 3.0 not found.")
  endif()
endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

target_include_directories(
  DBoW2 PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
               $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(
  TARGETS DBoW2
  EXPORT DBoW2Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES ${HDRS_DBOW2} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Thirdparty/DBoW2/DBoW2)
install(FILES ${HDRS_DUTILS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Thirdparty/DBoW2/DUtils)
install(
  EXPORT DBoW2Targets
  FILE DBoW2Targets.cmake
  NAMESPACE DBoW2::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/DBoW2
)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2Config.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/DBoW2Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/DBoW2
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/DBoW2Config.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/DBoW2
)
