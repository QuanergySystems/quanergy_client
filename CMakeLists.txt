project(QuanergyClient)
cmake_minimum_required(VERSION 2.8)

#  This should match what's in include/quanergy/client/version.h
set(QUANERGY_CLIENT_VERSION "1.0.0")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unknown-pragmas -fno-strict-aliasing -Wno-format-extra-args -Wno-sign-compare -Wno-invalid-offsetof -Wno-conversion")
endif()


# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
set(INSTALL_CMAKE_DIR share/quanergy_client CACHE PATH "Installation directory for CMake files")

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

find_package(PCL REQUIRED common io visualization)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS})

link_directories(
  ${Boost_LIBRARY_DIR}
  ${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

# There is no library. Only headers at this point.
#add_library(quanergy_client SHARED src/m8_client.cpp)

add_executable(test src/test.cpp)
target_link_libraries(test ${PCL_LIBRARIES})


configure_file(docs/doxyfile.in
  "${PROJECT_BINARY_DIR}/doxyfile" @ONLY)

find_package(Doxygen)
if(DOXYGEN_FOUND)
  add_custom_target(doc ALL "${DOXYGEN_EXECUTABLE}" "${PROJECT_BINARY_DIR}/doxyfile")
endif(DOXYGEN_FOUND)


#  This is the export CMake stuff form.
#set_target_properties(quanergy_client PROPERTIES SOVERSION ${QUANERGY_CLIENT_VERSION})
#install(TARGETS quanergy_client
#  EXPORT QuanergyClientTargets
#  LIBRARY DESTINATION "${INSTALL_LIB_DIR}")

install(DIRECTORY include/quanergy
  DESTINATION "${INSTALL_INCLUDE_DIR}")


# Add all targets to the build-tree export set
#export(TARGETS quanergy_client FILE "${PROJECT_BINARY_DIR}/QuanergyClientTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE QuanergyClient)

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

# No library yet
#  install(EXPORT QuanergyClientTargets DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

endif()
