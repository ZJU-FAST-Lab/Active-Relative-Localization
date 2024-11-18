# Active-Relative-Localization

We propose an active localization correction system designed to balance the contradiction between environmental observation and mutual observation within the FoV-limited drone swarm, ensuring the swarm's flight safety.

**[IROS 2024]** Open source code for paper **Preserving Relative Localization of FoV-Limited Drone Swarm via Active Mutual Observation**. [arXiv Preprint](https://arxiv.org/abs/2407.01292)

Video Links: [Bilibili](https://www.bilibili.com/video/BV1fZ421b7j2/).

image

## Quick Start

We developed the code on Ubuntu 20.04 with ROS Noetic. Theoretically, it should also run on Ubuntu 18.04 and Ubuntu 16.04, but this has not been tested. Please install ROS(Robot Operating System) first following http://www.ros.org. We recommend installing the full-desktop version.

Run the following commands to setup:

```
git clone https://github.com/ZJU-FAST-Lab/Active-Relative-Localization.git
cd Active-Relative-Localization
catkin_make
```

We provide a launch file to run the simulation. You can run the simulation with the following command:

**line formation**


```
source devel/setup.bash && roslaunch ego_planner rviz.launch 
source devel/setup.bash && roslaunch ego_planner line_formation.launch 
```

**x formation**

```
source devel/setup.bash && roslaunch ego_planner rviz.launch 
source devel/setup.bash && roslaunch ego_planner x_formation.launch 
```

If you want to compare the difference between using and not using active mutual observation, you can change the parameter ```fsm/active_obs``` to true in ```src/plan_manage/launch/line_formation/advanced_param.xml```
or ```src/plan_manage/launch/x_formation/advanced_param_x.xml```

```
<param name="fsm/active_obs" value="true" type="bool"/> 
```

## Acknowledgments

The simulation and trajectory planning code are based on our previous work [Ego-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2), and we use [MINCO](https://github.com/ZJU-FAST-Lab/GCOPTER) as our trajectory representation.