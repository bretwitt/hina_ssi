# Hina Soft-Soil Interaction (HSSI) Project (Under Construction)

Plugins for Gazebo 11 to simulate soft soil interaction with rigidbodies.

Requires CGAL (libcgal-dev) and Gazebo 11 

## Build instructions

Building (-s flag to skip CMake config)
```
chmod +x build.sh
./build.sh 
```

Load the test world with plugins AFTER building
```
chmod +x launch.sh
./launch.sh
```

.so files needed will be in the build folders, see ```gz_hina/test.world``` for example on how to use the .so files in your own world
