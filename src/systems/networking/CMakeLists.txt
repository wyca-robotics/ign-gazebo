#--------------------------------------
# Find Protobuf
set(REQ_PROTOBUF_VER 3)
ign_find_package(IgnProtobuf
                 VERSION ${REQ_PROTOBUF_VER}
                 REQUIRED
                 COMPONENTS all
                 PRETTY Protobuf)

add_subdirectory(msgs)
set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER}
                            PROPERTIES GENERATED TRUE)

set (network_system_sources
  ClientManager.cc
  NetworkPrimary.cc
  NetworkSecondary.cc
  ${PROTO_SRC}
)

ign_add_component(network-system
  SOURCES ${network_system_sources}
  GET_TARGET_NAME network_system_target
  CXX_STANDARD 17)

target_include_directories(${network_system_target}
  PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_dependencies(${network_system_target} protobuf_compilation)

target_link_libraries(${network_system_target}
  PUBLIC
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
  ignition-gazebo
  protobuf::libprotobuf
  PRIVATE
  ignition-plugin${IGN_PLUGIN_VER}::register
)

install(
  TARGETS ${network_system_target}
  DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR}
)
