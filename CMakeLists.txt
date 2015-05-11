project(M8Client)

cmake_minimum_required(VERSION 2.8)

if (${CMAKE_BUILD_TYPE} MATCHES "Debug")
  add_definitions("-DDEBUG")
endif (${CMAKE_BUILD_TYPE} MATCHES "Debug")

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
