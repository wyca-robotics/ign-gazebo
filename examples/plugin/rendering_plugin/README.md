# Rendering plugin

## Build

From the root of the `ign-gazebo` repository, do the following to build the example:

~~~
cd ign-gazebo/examples/plugins/rendering_plugin
mkdir build
cd build
cmake ..
make
~~~

This will generate the `libRenderingPlugin.so` library under `build`.

## Run

Add the library to the path:

~~~
cd ign-gazebo/examples/plugins/rendering_plugin
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then run a world that loads the plugin as follows:

    ign gazebo -s -v 4 rendering_plugin.sdf

