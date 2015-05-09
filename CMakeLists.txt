project(M8Client)

cmake_minimum_required(VERSION 2.8)

find_package(Boost COMPONENTS system thread chrono REQUIRED)

find_package(PCL REQUIRED common io)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS})

link_directories(
  ${Boost_LIBRARY_DIR}
  ${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_library(m8_client src/m8_client.cpp)
