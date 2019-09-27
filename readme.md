# simple_moveit_interface

This python class is a wrapper around [MoveIt's](https://moveit.ros.org) moveit_commander [python interface](http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html) to simplify the scripting of robotic applications with ROS and MoveIt.

## Offered methods

### Motions
- Execute a motion plan
- Move to a 6D pose
- Move to a 3D position
- Move to a predefined fixed pose
- Move to a named TF pose
- Move to the position of a named TF pose
- Move to a joint state
- Move in x/y/z direction
- Move in x/y/z direction in a straight line
- Rotate around x/y/z
- Move a cartesian path to pose


### Planning scene
- Initialize the planning scene
- Add a simple ground to the scene
- Add a box to the scene
- Wait for an object to be spawned


## Usage
See the example code in [test_moveit_interface.py](scripts/test_moveit_interface.py)

## Links
- MoveIt: https://moveit.ros.org
- Move Group Python interface: http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
MoveIt Commander documentation: http://docs.ros.org/melodic/api/moveit_commander/html/index.html
