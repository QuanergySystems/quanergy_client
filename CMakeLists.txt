project(M8Client)

cmake_minimum_required(VERSION 2.8)

set(VERSION "0.2.2")

if (${CMAKE_BUILD_TYPE} MATCHES "Debug")
  add_definitions("-DDEBUG")
endif (${CMAKE_BUILD_TYPE} MATCHES "Debug")

option(USE_CLIENT_TIME "If enabled, clouds will be time stamped by the M8 Client not the sensor" OFF)

if (USE_CLIENT_TIME)
  add_definitions("-DCLIENT_TIME")
endif (USE_CLIENT_TIME)

find_package(Boost COMPONENTS system thread chrono REQUIRED)

find_package(PCL REQUIRED common io visualization)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS})

link_directories(
  ${Boost_LIBRARY_DIR}
  ${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_library(m8_client SHARED src/m8_client.cpp)

add_executable(test src/test.cpp)
target_link_libraries(test m8_client ${PCL_LIBRARIES})

configure_file(docs/doxyfile.in
  "${PROJECT_BINARY_DIR}/doxyfile" @ONLY)

find_package(Doxygen)
if(DOXYGEN_FOUND)
  add_custom_target(doc ALL "${DOXYGEN_EXECUTABLE}" "${PROJECT_BINARY_DIR}/doxyfile")
endif(DOXYGEN_FOUND)
