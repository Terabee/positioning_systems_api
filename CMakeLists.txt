cmake_minimum_required(VERSION 3.12.0)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PACKAGE_VERSION 1.5.6)
project(positioning_systems_api VERSION ${PACKAGE_VERSION})

if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -O0 -g3 -pthread -fPIC")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -O2 -g0 -pthread -fPIC")
endif()

option(BUILD_TESTS "Builds tests" ON)
option(BUILD_EXAMPLES "Build short example binaries" ON)

include(GNUInstallDirs)
set(project_config_in "${CMAKE_CURRENT_LIST_DIR}/cmake/positioning_systems_apiConfig.cmake.in")
set(project_config_out "${CMAKE_CURRENT_BINARY_DIR}/positioning_systems_apiConfig.cmake")
set(positioning_systems_api_targets "positioning_systems_apiTargets")
set(version_config_file "${CMAKE_CURRENT_BINARY_DIR}/positioning_systems_apiConfigVersion.cmake")
set(export_dest_dir "lib/cmake/positioning_systems_api")

add_subdirectory(serial_communication)
add_subdirectory(follow_me_driver)
add_subdirectory(rtls_driver)
add_subdirectory(logger)
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()


#-------------- Install ---------------#
install(EXPORT ${positioning_systems_api_targets}
  DESTINATION "${export_dest_dir}"
  NAMESPACE positioning_systems_api::
  FILE "${positioning_systems_api_targets}.cmake"
  COMPONENT configs
)

include(CMakePackageConfigHelpers)
# configure_file("${project_config_in}" "${project_config_out}" @ONLY)
# generate the config file that is includes the exports
configure_package_config_file(${project_config_in} ${project_config_out}
  INSTALL_DESTINATION "${export_dest_dir}"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
  )

write_basic_package_version_file("${version_config_file}" COMPATIBILITY SameMajorVersion)

install(FILES
  "${project_config_out}"
  "${version_config_file}" DESTINATION "${export_dest_dir}"
)
