# v4l_ros2
ROS2 node for interfacing custom v4l nodes

## Build and run the node:
Before build, adapt the following parameters:
* width
* height
* control_device
  
Clone repository into ```ros2_ws/src/``` and run the following command in ```v4l_ros2/camera_node.py```:
```console
colcon build
source install/local_setup.bash
ros2 run v4l_ros2 camera_node
```
To start the stream, after setting the desired parameters, enable the "start_button" parameter (this can be done from rqt_reconfigure window or via command line)

## Parameter access
In new terminal run:
```console
ros2 run rqt_reconfigure rqt_reconfigure
```

## Strream visualizing
In new terminal run:
```console
ros2 run rqt_image_view rqt_image_view
```
