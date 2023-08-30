# EMC2_ros2_hw_interface
This is a child project of the Easy Motor Control (EMC2) project. This is to be used with **Ubuntu 22.04 (ros2 humble)** in your linux-based microcomputer **ROS2** mobile robotic project (as it depends on the libserial-dev linux package) to communicate with the EMC2_driver module in order to send target angular velocities to the motors or receive the motor's angular velocity and angular position, after successful velocity PID setup with the [EMC2_gui_apllication](https://github.com/samuko-things/EMC2_gui_application).

> **NOTE:** should be used with your microcomputer robotics project running on linux Ubuntu 22.04 [ROS2 Humble] (e.g Raspberry Pi, Nvida, PC, etc.)


### Quick Demo

![Demo Video](https://github.com/samuko-things/EMC2_ros2_hw_interface/blob/main/docs/emc2_hw_test.gif)

![Local Demo Video](./docs/emc2_hw_test.gif)


## How to Use the Package
- ensure you've already set up your microcomputer or PC system with ros2-humble with colcon and your ros workspace also setup

- install the libserial-dev package on your linux machine
  > sudo apt-get update
  >
  > sudo apt install libserial-dev

- In the src/ folder of your ros workspace, clone the repo (or you can download and add it manually to the src/ folder)
  > ```git clone https://github.com/samuko-things/EMC2_ros2_hw_interface.git```
  >
  > **NOTE:** the cloned or downloaded file comes with two packages
  > (one is the **emc2_ros2_arduino_driver_interface** package and the other is **my_diff_bot** package)
  > - the **emc2_ros2_arduino_driver_interface** is the actual hardware interface for the EMC2_driver shield
  > - the **my_diff_bot** package is a test robot used as a quick test to see if the driver and the ros interface are working together properlly. 

- build the packages with colcon (in your ros workspace root folder):
  > ```colcon build --packages-select emc2_ros2_arduino_driver_interface my_diff_bot --symlink-install```
  >
  > **NOTE:** the **emc2_ros2_arduino_driver_interface** hardware interface will now be available for use in any project in your ros workspace. 
  > check the sample_ros2_control_file in the **emc2_ros2_arduino_driver_interface** pkg file to see the sample xacro file you can use in your own URDF controller file (also view the **my_diff_bot** pkg file to see how it was used in the URDF file)

## Test with my_diff_bot pkg (Similar to the short video Above)
- ensure the EMC2_driver module shield (with the motors connected and fully set up for velocity PID) is connected to the microcomputer or PC via USB.

- check the serial port the driver is connected to:
  > The best way to select the right serial port (if you are using multiple serial device) is to select by path
  >
  > ```ls /dev/serial/by-path```
  >
  > you should see a value (if the driver is connected and seen by the computer), your serial port would be -> /dev/serial/by-path/[value]. for more info visit this tutorial from [ArticulatedRobotics](https://www.youtube.com/watch?v=eJZXRncGaGM&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=8)

  - OR you can also try any of the following below:

  > if you're using the Arduino NANO shield, check the usb port using:
  >
  > ```ls /dev/ttyU*```
  >
  > you should see /dev/ttyUSB0 or /dev/ttyUSB1 and so on


  > if you're using the Arduino UNO shield, check the usb port using:
  >
  > ```ls /dev/ttyA*```
  >
  > you should see /dev/ttyACM0 or /dev/ttyACM1 and so on


- open the **my_diff_bot**/urdf/ros_controllers.xacro file and change the port param value to the value gotten from the previous step and then rebuild the package

- open a new terminal and source your ros workspace and run:
  >```ros2 launch my_diff_bot my_diff_bot.launch.py```
  >
  > you should see the robot rviz view

- open another terminal and source your ros workspace and run:
  >```ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped```

- as you drive the robot should start moving.

- this means you can use it in your project.