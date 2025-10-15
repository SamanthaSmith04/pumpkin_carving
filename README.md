# AIMS Pumpkin Carving Motion Planning + UI

## Workspace Setup

```
mkdir aims_pumpkin_carving
cd aims_pumpkin_carving

mkdir -p pumpkin_ws/src
```
Clone this repo into `pumpkin_ws/src`

### Tesseract Setup
```
mkdir -p tesseract_ws/src
vcs import tesseract_ws/src < pumpkin_ws/src/aims_pumpkin/dependencies.repos
vcs import tesseract_ws/src < tesseract_ws/src/dependencies.repos # this will take a while

rosdep install --from-paths src -iry

cd tesseract_ws
colcon build --symlink-install --cmake-args -DTESSERACT_BUILD_FCL=OFF -DBUILD_RENDERING=OFF -DBUILD_STUDIO=OFF
```

### Dependencies
```
mkdir -p pumpkin_deps_ws/src
vcs import pumpkin_deps_ws/src < pumpkin_ws/src/aims_pumpkin/dependencies.repos

rosdep install --from-paths src -iry

cd pumpkin_deps_ws
colcon build
```

### Building Pumpkin
```
cd pumpkin_ws
source ../tesseract_ws/install/setup.bash
source ../pumpkin_deps_ws/install/setup.bash
colcon build
source install/setup.bash
``` 
## Running the Application

```
ros2 launch aims_pumpkin  motion_planning_server.launch
```

This will launch the motion planning server and RViz with the Pumpkin BT Application as a panel plugin. Currently motion plans can be loaded from a YAML file and executed on a real robot.

## Parameters 
There are several parameters exposed via ROS2 parameters that can be used to tune the motion planning server behavior. These can be set through RQT > Plugins > Configuration > Dynamic Reconfigure > /motion_planning_server or by passing in parameters via the command line when launching the server.