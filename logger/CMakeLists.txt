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

add_library(${LIB_NAME} SHARED
  src/logger.cpp
)

target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/spdlog-src/include/>
  $<INSTALL_INTERFACE:include/>
)

target_link_libraries(${LIB_NAME} ${MODULE_LIBS})

install(DIRECTORY include/ ${CMAKE_BINARY_DIR}/spdlog-src/include/
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  COMPONENT headers
)

install(TARGETS ${LIB_NAME}
  EXPORT ${positioning_systems_api_targets}
  DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  COMPONENT libraries
)
