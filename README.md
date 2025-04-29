# MoveIt

## MoveIt quickstart tutorial

Here are the instructions to set-up and run the basic [MoveIt Quickstart in Rviz tutorial](https://ros-planning.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html#)

Installation:

```sh
sudo apt install ros-humble-moveit ros-humble-moveit-visual-tools # ros-noetic-panda-moveit-config 
```

Currently there is a bug in the tutorials: the `franka_ros` model changed but this is not (yet) reflected in ros noetic moveit packages.
As written in [CHANGELOG](https://github.com/frankaemika/franka_ros/blob/71ff2e06cdbe3ebb51ef67933dc068909ba9dec9/CHANGELOG.md) in [franka_ros](https://github.com/frankaemika/franka_ros) (0.8.0 - 2021-08-03):

> BREAKING Remove panda_arm_hand.urdf.xacro. Use panda_arm.urdf.xacro hand:=true instead.

To fix the issue, edit `/opt/ros/noetic/share/panda_moveit_config/launch/planning_context.launch` (as root) and replace:

```xml
<!-- Load universal robot description format (URDF) -->
<param if="$(eval arg('load_robot_description') and arg('load_gripper'))" name="$(arg robot_description)" command="$(find xacro)/xacro '$(find franka_description)/robots/panda_arm.urdf.xacro' hand:=true"/>
<param if="$(eval arg('load_robot_description') and not arg('load_gripper'))" name="$(arg robot_description)" command="$(find xacro)/xacro '$(find franka_description)/robots/panda_arm.urdf.xacro'"/>
``` 

by

```xml
<!-- Load universal robot description format (URDF) -->
<param name="$(arg robot_description)" command="$(find xacro)/xacro $(find franka_description)/robots/panda_arm.urdf.xacro hand:=$(arg load_gripper)" />
```

Alternatively you can [install MoveIt from source](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html) and ignore the fix above.

Then follow the [MoveIt Quickstart in Rviz tutorial](https://ros-planning.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html#).

