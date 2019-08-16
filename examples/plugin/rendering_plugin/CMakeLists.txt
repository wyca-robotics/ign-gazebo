cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(RenderingPlugin)

find_package(ignition-cmake2 REQUIRED)
find_package(ignition-gazebo2 REQUIRED)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-rendering2 QUIET REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering2_VERSION_MAJOR})

add_library(RenderingPlugin SHARED RenderingPlugin.cc)
set_property(TARGET RenderingPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(RenderingPlugin
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-rendering${IGN_RENDERING_VER}::core
  PRIVATE ignition-gazebo2::ignition-gazebo2)
