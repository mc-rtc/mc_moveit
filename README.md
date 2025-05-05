# mc_moveit 

mc_moveit is a simple interface between the [mc_rtc](https://github.com/jrl-umi3218/mc_rtc) and ros' [MoveIt](https://moveit.picknik.ai).

At it's heart it provides a `mc_moveit::Planner` interface whose role is to initialize the various planners, add collision shapes/objects (all rbdyn types supported), provide targets and plan trajectories. Upon success, it provides a `mc_moveit::Trajectory` result that can be automatically executed with the provided tasks, or you can write your own control.

For more detailed use, please refer to the [src/sample.cpp](./src/sample.cpp) controller. You can run it using:

```sh
./mc_moveit_sample
```

and visualize it with

```sh
ros2 launch mc_rtc_ticker display.launch
```
