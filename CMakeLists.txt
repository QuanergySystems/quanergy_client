project(QuanergyClient)
cmake_minimum_required(VERSION 2.8)

set(QUANERGY_CLIENT_VERSION "1.0.0")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if (${CMAKE_BUILD_TYPE} MATCHES "Debug")
  add_definitions("-DDEBUG")
endif (${CMAKE_BUILD_TYPE} MATCHES "Debug")

if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unknown-pragmas -fno-strict-aliasing -Wno-format-extra-args -Wno-sign-compare -Wno-invalid-offsetof -Wno-conversion")
endif()

find_package(Boost COMPONENTS system thread REQUIRED)

find_package(PCL REQUIRED common io visualization)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS})

link_directories(
  ${Boost_LIBRARY_DIR}
  ${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(test src/test.cpp)
target_link_libraries(test ${PCL_LIBRARIES})
