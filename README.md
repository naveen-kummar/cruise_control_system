# cruise_control

This project is created for development and review of cruise control system in ROS2 platform and compatible in Docker environment

Please make sure you have followed the steps in below link for installing ROS2 and Dependencies

https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup/
https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/


## Build

Clone this repository in your ROS2 workspace.
```bash
cd ~/ros2_example_ws/src && https://gitlab-public.kpit.com/cruise_control_group/cruise_control.git
```

Compile.
```bash
source /opt/ros/dashing/setup.bash
cd ~/ros2_example_ws && colcon build --symlink-install
```

## Run

### Terminal 1

Run the Vehicle Stub Node

```
source ~/ros2_example_ws/install/setup.bash
ros2 run cruise_controller vehicle_stub 
```

### Terminal 2

In a different terminal, run the Cruise controller Node and you have option to pass set_speed value.

```
source ~/ros2_example_ws/install/setup.bash
ros2 run cruise_controller cruise_manager set_speed
```

You will provided with option to change other default paramater values of Vehicle Mass, Resistance Coefficient and Driving Force through console.

If you wish to change then press 'y' or 'Y' to change the default values.

