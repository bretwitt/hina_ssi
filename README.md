# TaRO-SCM Project (Under Construction)

Plugins for Gazebo 11 to simulate soft soil interaction with rigidbodies.

## Dependencies

Requires [GDAL](https://gdal.org/) and [libInterpolate](https://github.com/CD3/libInterpolate)

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

This plugin uses OGRE materials, append to  your ~/.bashrc the directory of the Hina/Soil material file, the gz_hina directory is optional as long as it points to a Hina/Soil material 
```
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:PATH/TO/REPO/gz_hina/media/materials:PATH/TO/REPO/gz_hina/media/texture
```

To use in your Gazebo world after building, it is recommended that you append to your ~/.bashrc
```        
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:PATH/TO/REPO/build/src/hina_ssi_physics:PATH/TO/REPO/build/src/hina_ssi_visual
```



CubeRover on Lunar Regolith

<img src="https://user-images.githubusercontent.com/14039800/193426261-92fd8d30-0efa-45d7-9ba5-f4dc6935d1c7.png"/>
