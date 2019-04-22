if(WIN32)
  # when using vcpkg in Windows the call to IgnFindProtobuf
  # does not find the macro PROTOBUF_GENERATE_CPP
  find_package(Protobuf)
endif()

PROTOBUF_GENERATE_CPP(PROTO_PRIVATE_SRC PROTO_PRIVATE_HEADERS
  peer_info.proto
  peer_control.proto
  performer_affinity.proto
  simulation_step.proto
)

set(PROTO_PRIVATE_SRC ${PROTO_PRIVATE_SRC} PARENT_SCOPE)
set(PROTO_PRIVATE_HEADERS ${PROTO_PRIVATE_HEADERS} PARENT_SCOPE)

add_custom_target(ignition-gazebo_private_msgs
  DEPENDS ${PROTO_PRIVATE_SRC}
)

