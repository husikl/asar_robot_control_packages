# asar_robot_control_packages
Surgical assistant robot packages for PSM with Kinova Gen3 Arm and OpenRST surgical tool. DOI: [10.1109/ACCESS.2023.3236821](https://doi.org/10.1109/ACCESS.2023.3236821).

## Maintainers
- Jacinto Colan: [Email](mailto:colan@robo.mein.nagoya-u.ac.jp)
- Khusniddin Fozilov: [Email](mailto:khusniddin@mein.nagoya-u.ac.jp)
## Packages

### asar_control
This package contains a ROS controller for the Autonomous Surgical Assistant Robot (ASAR). It makes use of the Kinova Gen3 control package and the articulated forceps control package.

#### Testing Nodes

- `asar_planner.cpp`: Sample action class that provides basic kinematic function (IK, FK). Still in development.
- `asar_teleop.cpp`: Provides teleoperation control following a pose target obtained from the Virtuose haptic interface. Includes simulation environment.
- `asar_clrmpc.cpp`: Provides autonomous control by reading a pose target from the "target.csv" file. The target is written by the Conditional Latent regressor MPC controller.

### asar_description
This package contains the URDF (Unified Robot Description Format), STL and configuration files for the Autonomous Surgical Assistant Robot (ASAR). The ASAR robot comprises:

- Kinova Gen3 arm (7DOF)
- Articulated forceps (3DOF)

#### Tool frame
The `ee_link` link refers to the tool frame used by the robot when it reports end effector position feedback. It refers to the center of the articulated forceps gripper.

### codcs_ik
This package contains the COnstrained Dual Concurrent Solver for Inverse Kinematic (CODCS-IK). It is based on the simultaneous execution of two IK solvers:

- Hierarchical Task-Priority IK
- Nonlinear Optimization-based IK

A newer version, with multiple concurrent IK solvers, is available at [CoIKS](https://github.com/jcolan/CoIKS.git).

### forceps_control
This package contains the ROS controller of the 3-DOF articulated forceps (gripper pitch, yaw, open/close). It is based on the ros-control package.

#### Joints
- `pitch_joint`
- `finger_left_joint`
- `finger_right_joint`

### gen3_control
This package contains source code and configuration files to control the Kinova Gen3 7DOF with ROS. It provides access to the Kortex low-level API for joint position control. It is based on a previous implementation for controlling a DENSO VS050.

#### How to use it
Follow the instructions in [vs050_control](https://github.com/jcolan/vs050_control).

### mc_daq_ros
This package contains source code and configuration files to support the Measurement Computing Data Acquisition Boards. Used to communicate with the end effector. It supports the following USB boards:

- USB1608: AI DAQ Board
- USB3104: AO DAQ Board
- USBQUAD08: Encoder/Counter Board
