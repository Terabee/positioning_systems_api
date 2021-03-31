project(positioning_systems_api_examples)

add_executable(serial_example
  serial_example.cpp
)

target_link_libraries(serial_example
  serial_communication
)

add_executable(follow_me_master_example
  follow_me_master_example.cpp
)

target_link_libraries(follow_me_master_example
  follow_me_driver
  serial_communication
)

add_executable(follow_me_remote_example
  follow_me_remote_example.cpp
)

target_link_libraries(follow_me_remote_example
  follow_me_driver
  serial_communication
)

add_executable(rtls_anchor_example
  rtls_anchor_example.cpp
)

target_link_libraries(rtls_anchor_example
  rtls_driver
  serial_communication
)

add_executable(rtls_tracker_example
  rtls_tracker_example.cpp
)

target_link_libraries(rtls_tracker_example
  rtls_driver
  serial_communication
)
