# 🎃 AIMS Pumpkin Carving Support Package 🎃

## Workspace Setup

```bash
mkdir aims_pumpkin_carving
cd aims_pumpkin_carving

mkdir -p pumpkin_ws/src
```
Clone this repo into `pumpkin_ws/src`

### Tesseract Setup 
```bash
mkdir -p tesseract_ws/src
vcs import tesseract_ws/src < pumpkin_ws/src/pumpkin_carving/aims_pumpkin/dependencies.repos
vcs import tesseract_ws/src < tesseract_ws/src/tesseract_ros2/dependencies.repos # this will take a while

cd tesseract_ws

# ** this step will require sudo permissions **
rosdep install --from-paths src -iry

colcon build --symlink-install --cmake-args -DTESSERACT_BUILD_FCL=OFF -DBUILD_RENDERING=OFF -DBUILD_STUDIO=OFF

cd ..
```

### Dependencies
```bash
mkdir -p pumpkin_deps_ws/src
vcs import pumpkin_deps_ws/src < pumpkin_ws/src/pumpkin_carving/dependencies.repos
vcs import pumpkin_deps_ws/src < pumpkin_deps_ws/src/motoros2_client_interface_dependencies/source_deps.repos 


cd pumpkin_deps_ws

# ** this step will require sudo permissions **
rosdep install --from-paths src -iry

colcon build

cd ..
```

### Building Pumpkin
```bash
cd pumpkin_ws
source ../tesseract_ws/install/setup.bash
source ../pumpkin_deps_ws/install/setup.bash
colcon build
source install/setup.bash
``` 
## Running the Application

```bash
ros2 launch aims_pumpkin  motion_planning_server.launch
```

This will launch the motion planning server and RViz with the Pumpkin BT Application as a panel plugin. Currently motion plans can be loaded from a YAML file and executed on a real robot.

## Parameters 
There are several parameters exposed via ROS2 parameters that can be used to tune the motion planning server behavior. These can be set through RQT > Plugins > Configuration > Dynamic Reconfigure > /motion_planning_server or by passing in parameters via the command line when launching the server.

<br>
<br>

# Calling the Motion Planning Service
## Client Overview
The path planner client will need to send over a service request of type: `pumpkin_msgs/srv/PlanMotion.srv` to the service `/generate_motion_plan`. The only thing you will need to fill out for the request is a list of [PoseArray](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseArray.html). It's up to you if you want to split them into multiple pose arrays, or send it all as one.

## Processing Pose Arrays
The motion planner will process the paths in this order:

`FREESPACE` from current position to first pose array position
`LINEAR` motion through each point in the pose array
`FREESPACE` from last position to first postion of next pose array
.
.
.
`FREESPACE` back to the `home` positon (currently all 0s joint states)

## Service Response
The service will respond with a `pumpkin_msgs/srv/PlanMotion_Response` message, which, if successful, will contain a `tesseract_msgs/msg/JointTrajectory` message that can be executed on the robot. The client will need to save this trajectory message to a YAML file to be loaded into the UI for execution.

### Using the YAML Utility from the Client C++

A utility already exists for C++ in [`aims_pumpkin/include/aims_pumpkin/yaml_utils.h`](https://github.com/SamanthaSmith04/pumpkin_carving/blob/main/aims_pumpkin/include/aims_pumpkin/yaml_utils.h) that can be used to save the trajectory message to a YAML file. (either include this package in your CMakeLists or copy the file into your include folder and include the CMakeLists updates below)

```cpp
#include "yaml_utils.h"
#include <tesseract_msgs/msg/joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

tesseract_msgs::msg::JointTrajectory trajectory_msg; // this would be filled out from the service response

YAML::Node yaml;
try {
  yaml = YAML::convert<tesseract_msgs::msg::JointTrajectory>::encode(trajectory_msg);
}
catch (const std::exception& e) {
  RCLCPP_ERROR(rclcpp::get_logger("yaml_utils"), "Failed to convert trajectory message to YAML: %s", e.what());
  return;
}

std::ofstream fout("path_to_save_trajectory.yaml");
fout << yaml;
fout.close();
```

(If you copied yaml_utils.hpp into your package) In your `CMakeLists.txt` (replace project name with the correct target):
```cmake
find_package(yaml-cpp REQUIRED)

ament_target_dependencies(<PROJECT_NAME> yaml-cpp)

target_link_libraries(<PROJECT_NAME>
    yaml-cpp
)
```
  
If your client implementation uses Python, another utility exists from the part line grinding project that can be found [here](https://github.com/OSU-AIMS/012350-pl-grind/blob/main/plg_test/scripts/export_utility.py).

### Using the YAML Utility from the Client Python
Move the export_utility.py file into your Python client package and use the following code snippet to save the trajectory message to a YAML file:
```python
import yaml
from tesseract_msgs.msg import JointTrajectory

trajectory_msg = JointTrajectory()  # this would be filled out from the service response

ExportUtility("log").write_joint_trajectory_to_yaml(trajectory_msg)
```

## Additional Considerations
  - The motion planner assumes z is pointing into the pumpkin, so please make sure your path plan uses this orientation
