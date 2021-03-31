project(logger)
set(LIB_NAME logger)

# Download and unpack spdlog at configure time
configure_file(spdlog_download.in ${CMAKE_BINARY_DIR}/spdlog-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/spdlog-download )
if(result)
    message(FATAL_ERROR "CMake step for spdlog failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/spdlog-download )
if(result)
    message(FATAL_ERROR "Build step for spdlog failed: ${result}")
endif()

add_subdirectory(${CMAKE_BINARY_DIR}/spdlog-src
  ${CMAKE_BINARY_DIR}/spdlog-build
  EXCLUDE_FROM_ALL
)

set(MODULE_LIBS spdlog_header_only)
set_target_properties(spdlog_header_only PROPERTIES EXPORT_NAME "positioning_systems_api_spdlog_header_only")

add_library(${LIB_NAME}
  src/logger.cpp
)

target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:include/>
)

target_link_libraries(${LIB_NAME} ${MODULE_LIBS})

install(DIRECTORY include/
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  COMPONENT headers
)

install(TARGETS ${LIB_NAME} spdlog_header_only
  EXPORT ${LIB_NAME}
  DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  COMPONENT libraries
)

install(EXPORT ${LIB_NAME}
  DESTINATION "${export_dest_dir}"
  NAMESPACE positioning_systems_api::
  FILE ${config_targets_file_logger}
  COMPONENT configs
)
