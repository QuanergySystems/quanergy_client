project(QuanergyClient)
cmake_minimum_required(VERSION 2.8)

#  This should match what's in include/quanergy/client/version.h
set(QUANERGY_CLIENT_VERSION "3.0.2")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

option(NoViz "Do not build visualizer" OFF)

if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unknown-pragmas -fno-strict-aliasing -Wno-format-extra-args -Wno-sign-compare -Wno-invalid-offsetof -Wno-conversion")
endif()


# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
set(INSTALL_CMAKE_DIR share/${PROJECT_NAME} CACHE PATH "Installation directory for CMake files")

option(PACKAGE_FOR_DEV "Create -dev package" ON)
option(BUILD_APPS "Build applications" OFF)


# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()


SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${INSTALL_LIB_DIR}")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${INSTALL_LIB_DIR}" is_system_dir)
IF("${is_system_dir}" STREQUAL "-1")
  SET(CMAKE_INSTALL_RPATH "${INSTALL_LIB_DIR}")
ENDIF("${is_system_dir}" STREQUAL "-1")



find_package(Boost COMPONENTS system thread REQUIRED)

if (NoViz OR CMAKE_CROSSCOMPILING)
  find_package(PCL REQUIRED common io)
else()
  find_package(PCL REQUIRED common io visualization)
endif()

file(GLOB_RECURSE project_HEADERS
    "*.h"
    "*.hpp"
)


include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS})

link_directories(
  ${Boost_LIBRARY_DIR}
  ${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


set(client_SRCS
  src/modules/polar_to_cart_converter.cpp
  src/modules/distance_filter.cpp
  src/modules/ring_intensity_filter.cpp
  src/modules/encoder_angle_calibration.cpp
  src/common/point_xyz.cpp
  src/common/point_xyzir.cpp
  src/client/failover_client.cpp
  src/parsers/data_packet_parser_00.cpp
  src/parsers/data_packet_parser_01.cpp
  src/parsers/data_packet_parser_failover.cpp
  src/parsers/data_packet_parser_m8.cpp
  ${project_HEADERS}
)


add_library(quanergy_client SHARED ${client_SRCS})

if(WIN32)
  target_link_libraries(quanergy_client ws2_32)
endif()

configure_file(doxyfile.in
  "${PROJECT_BINARY_DIR}/doxyfile" @ONLY)

#############
## Testing ##
#############

# Unit Tests
find_package(GTest)

if (GTEST_FOUND)
  add_executable(test_quanergy_client test/test_encoder_angle_calibration.cpp)

  target_link_libraries(test_quanergy_client
    quanergy_client
    ${GTEST_LIBRARIES}
    boost_system
    pthread
    )

  add_test(encoder_calibration_unit_test test_quanergy_client)
endif()

find_package(Doxygen)
if(DOXYGEN_FOUND)
  add_custom_target(doc "${DOXYGEN_EXECUTABLE}" "${PROJECT_BINARY_DIR}/doxyfile")
endif(DOXYGEN_FOUND)


#  This is the export CMake stuff form.

if (PACKAGE_FOR_DEV)

  install(DIRECTORY include/quanergy
    DESTINATION "${INSTALL_INCLUDE_DIR}" COMPONENT dev)

  if(WIN32)
  install(TARGETS quanergy_client
    EXPORT QuanergyClientTargets
    RUNTIME DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib)
  else()
  install(TARGETS quanergy_client
    EXPORT QuanergyClientTargets
    LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib)
  endif()

else()

  set_target_properties(quanergy_client 
    PROPERTIES SOVERSION ${QUANERGY_CLIENT_VERSION})

  if(WIN32)
  install(TARGETS quanergy_client
    EXPORT QuanergyClientTargets
    RUNTIME DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib NAMELINK_SKIP)
  else()
  install(TARGETS quanergy_client
    EXPORT QuanergyClientTargets
    LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib NAMELINK_SKIP)
  endif()

endif()


# Add all targets to the build-tree export set
export(TARGETS quanergy_client FILE "${PROJECT_BINARY_DIR}/QuanergyClientTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)

# Which of these is correct? The latter follows the pattern set by
# quanergy_core.

# export(PACKAGE QuanergyClient)
export(PACKAGE quanergy_client)

# Create the QuanergyClientConfig.cmake and QuanergyClientConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" ${INSTALL_INCLUDE_DIR})


# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
configure_file(cmake/QuanergyClientConfig.cmake.in
  "${PROJECT_BINARY_DIR}/QuanergyClientConfig.cmake" @ONLY)

# ... for the install tree
set(CONF_INCLUDE_DIRS "\${QUANERGY_CLIENT_CMAKE_DIR}/${REL_INCLUDE_DIR}" "${PCL_INCLUDE_DIRS}")
configure_file(cmake/QuanergyClientConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/QuanergyClientConfig.cmake" @ONLY)

# ... for both
configure_file(cmake/QuanergyClientConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/QuanergyClientConfigVersion.cmake" @ONLY)

if (PACKAGE_FOR_DEV)
  # Install the QuanergyClientConfig.cmake and QuanergyClientConfigVersion.cmake

  message("config: ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/QuanergyClientConfig.cmake")

  install(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/QuanergyClientConfig.cmake"
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/QuanergyClientConfigVersion.cmake"
    DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

  install(EXPORT QuanergyClientTargets DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

endif()

set(CPACK_GENERATOR "DEB")
if(PACKAGE_FOR_DEV)
  set(CPACK_PACKAGE_FILE_NAME "quanergy-client-${QUANERGY_CLIENT_VERSION}-dev")
  set(CPACK_DEBIAN_PACKAGE_NAME "quanergy-client-dev")
  set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "Quanergy client library - development")
  set(CPACK_DEBIAN_PACKAGE_SECTION libdevel)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "libpcl-common-1.7-dev (>= 1.7.0), libpcl-io-1.7-dev (>= 1.7.0), libboost-dev (>= 1.54), libboost-system1.54.0, quanergy-client")
  configure_file(debian/postinst-dev.in "${PROJECT_BINARY_DIR}/postinst")
  configure_file(debian/preinst-dev.in "${PROJECT_BINARY_DIR}/preinst")
  configure_file(debian/postrm-dev.in "${PROJECT_BINARY_DIR}/postrm")
  set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${PROJECT_BINARY_DIR}/preinst;${PROJECT_BINARY_DIR}/postinst;${PROJECT_BINARY_DIR}/postrm;")
else()
  set(CPACK_PACKAGE_FILE_NAME "quanergy-client-${QUANERGY_CLIENT_VERSION}")
  set(CPACK_DEBIAN_PACKAGE_NAME "quanergy-client")
  set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "Quanergy client library - runtime")
  set(CPACK_DEBIAN_PACKAGE_SECTION libs)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "quanergy-client (>= 0.1.0), libpcl-common-1.7 (>= 1.7.0), libpcl-io-1.7 (>= 1.7.0), libboost-system1.54.0")
  configure_file(debian/postinst.in "${PROJECT_BINARY_DIR}/postinst")
  configure_file(debian/prerm.in "${PROJECT_BINARY_DIR}/prerm")
  set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${PROJECT_BINARY_DIR}/postinst;${PROJECT_BINARY_DIR}/prerm;")
endif()
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Quanergy Systems Inc.") #required
set(CPACK_DEBIAN_PACKAGE_VERSION ${QUANERGY_CLIENT_VERSION})
if ("x86_64-linux-gnu" STREQUAL "${CMAKE_LIBRARY_ARCHITECTURE}")
  set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
else()
  set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "i386")
endif()
include(CPack)

##########
#  apps  #
##########

if (PCL_VISUALIZATION_FOUND)
  add_executable(visualizer apps/visualizer.cpp apps/visualizer_module.cpp)
  target_link_libraries(visualizer quanergy_client ${PCL_LIBRARIES} pthread)
endif()

message("PCL_LIBRARIES: ${PCL_LIBRARIES}")
