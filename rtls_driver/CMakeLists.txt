project(rtls_driver)
set(LIB_NAME rtls_driver)

add_library(${LIB_NAME} SHARED
  src/rtls_driver.cpp
)

target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
  $<INSTALL_INTERFACE:include/>
)

target_link_libraries(${LIB_NAME}
  serial_communication
  logger
)

set_target_properties(${LIB_NAME} PROPERTIES
  VERSION ${PACKAGE_VERSION}
)

install(DIRECTORY include/
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  COMPONENT headers
)

install(TARGETS ${LIB_NAME}
  EXPORT ${LIB_NAME}
  DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  COMPONENT libraries
)

install(EXPORT ${LIB_NAME}
  DESTINATION "${export_dest_dir}"
  NAMESPACE positioning_systems_api::
  FILE ${config_targets_file_rtls_driver}
  COMPONENT configs
)
