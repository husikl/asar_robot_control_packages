# ASAR Control

This package contains a ROS contrller for the Autonomous Surgical Assistant Robot (ASAR). It makes use of the [Kinova Gen3 control package](https://gitlab.com/nu_crest/robot_control/gen3_control) and the [articulated forceps control package](https://gitlab.com/nu_crest/robot_control/forceps_control).


## Testing Nodes

* asar_planner.cpp: Sample action class that provides basic kinematic function (IK, FK). Still in development
* asar_teleop.cpp: Provides teleoperation control folling a pose target obtained from the Virtuose haptic interface. Includes simulation environment.
* asar_clrmpc.cpp: Provides autonomous control by reading a pose target from the "target.csv" file. The target is written by the Conditional Latent regressor MPC controller.
