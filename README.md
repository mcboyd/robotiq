# Robotiq

This repo is a merger of the Danfoa Robotiq driver and the ROS-Industrial driver.  Gripper control is provided by
Danfoa, as we found it was better-documented and did what we wanted more reliably.  Force-torque sensor support is
from ROS-Industrial, as these were never supported by the Danfoa driver.

It appears that Robotiq themselves no longer support the ROS-Industrial package, hence the need for this fork.
See: https://github.com/ros-industrial/robotiq/issues/185#issuecomment-784918238

The contents of this README have not been properly-curated yet, so please forgive any missing/incorrect information.

For practical purposes, this package should work on Kinetic & Melodic.  We have no plans to maintain it for older ROS
distributions at this time.

## This Fork
This fork of the Clearpath original repo includes support for the Hand-E gripper attached to a UR e-series arm. This code was updated on ROS Melodic and may be backwards compatible with Kinetic, but this has not been tested. 

### Files w/ Hand-E Updates
See comments labeled with "MCB" in all files for specifics.
- *robotiq_modbus_rtu/src/robotiq_modbus_rtu/comModbusRtu.py*
  - Updated to address periodic timeouts and unusual responses
- *robotiq_2f_gripper_control/src/robotiq_2f_gripper_control/robotiq_2f_gripper_driver.py*
  - Updated in several locations to accomodate specifics of Hand-E grippers...look for "MCB" comments
- *robotiq_2f_gripper_control/scripts/robotiq_2f_action_server.py*
  - Updated to return the **current** gripper status at the end of a move, rather than the cached value from the last check

### New File for Hand-E Attached to UR5e
 - *robotiq_2f_gripper_control/launch/ur5e_hande_action_server.launch*
   - Sets defaults specific for communicating with the Hand-E using the default UR tool communication port */tmp/ttyUR*
   - Also sets the Hand-E joint name to report from URDF *hande_left_finger_joint*

## ROS Distro Support

|         | Kinetic | Melodic |
|:-------:|:-------:|:-------:|
| Branch  | [`kinetic-devel`](https://github.com/mcboyd/robotiq/tree/kinetic-devel) | [`melodic-hande`](https://github.com/mcboyd/robotiq/tree/melodic-hande) |)
| Status  |  supported |  supported |

## Support
[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

[ROS-Industrial][] robotiq meta-package.  See the [ROS wiki][] page for more information.

## License

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Contents

This repo holds source code for all versions > groovy. 

[ROS-Industrial]: http://www.ros.org/wiki/Industrial
[ROS wiki]: http://ros.org/wiki/robotiq
