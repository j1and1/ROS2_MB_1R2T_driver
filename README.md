# ROS 2 driver implementation for MB_1R2T lidar

This is a C++ implementation of MB_1R2T driver for ROS 2. 

## How to use?

To use this driver run `ros2 launch mb_1r2t_lidar_driver lidar_view.launch.py` to launch driver with rviz2 or `ros2 launch mb_1r2t_lidar_driver lidar_scan.launch.py`.
This driver has following parameters:
- `device` - serial port on which the MB_1R2T lidar is located. Default `/dev/ttyUSB0`
- `baud_rate` - specifies serial port baud rate. Default `153600`
- `frame_id` - specifies the frame id under which the publisher will send messages. Default `laser` 

## Devcontainer

To run the devcontainer in VSCode you need to attach lidar to the USB port and specify serial in `.devcontainer/devcontainer.json` if you would also like to run rviz2 in the container or other GUI apps you need to run `xhost +` from terminal on your host system. Debugging and build tasks are configured in VSCode. 
**NOTE:** after initial build you might want to source the environment

![Lidar in box](/images/lidar_in_the_box.png)