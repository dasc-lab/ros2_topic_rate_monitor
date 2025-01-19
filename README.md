# ROS2 Topic Rate Monitor

A small and simple utility to monitor the rate at which specific ros topics are being published. 

## Installation

Simply clone this repo into your `colcon_ws`, and then build the packages. 
You will need the following python dependencies: `PyYAML`, `prettytable`, and `numpy`. I used `PyYAML==6.0.2, prettytable==3.12.0, numpy==1.26.4` with `python==3.10.2`.

## Usage

In the config file, you specify the topic name, the topic type, the expected rate, and optionally the tolerance (defaults to 10%), and the window (defaults to 10 messages). 

Example `topics.yaml`:
```
topics:
  - name: /camera/infra1/camera_info
    type: sensor_msgs/msg/CameraInfo
    expected_rate: 15.0
    tolerance: 0.10
  - name: /visual_slam/tracking/vo_pose_covariance
    type: geometry_msgs/msg/PoseWithCovarianceStamped
    expected_rate: 30.0
    tolerance: 0.10
    window: 20
```
You can use any message type, as long as you can `ros2 topic echo <topic_name>` for that topic. 

Run the node, and specify the path to the config file as a ros2 parameter.
```
ros2 run topic_rate_monitor topic_rate_monitor --ros-args -p topics:=topics.yaml
```

It will print a table like 
```
[INFO] [1737250855.751291901] [topic_rate_monitor]: 
+------------------------------------------+--------------------+--------------------+-------------+---------------------+---------------------+
|                Topic Name                | Expected Rate (Hz) | Observed Rate (Hz) | Jitter (Hz) | Time since last (s) |        Status       |
+------------------------------------------+--------------------+--------------------+-------------+---------------------+---------------------+
| /visual_slam/tracking/vo_pose_covariance |    15.00 +- 10%    |       14.08        |     1.00    |         0.04        |          OK         |
|        /camera/infra1/camera_info        |    15.00 +- 10%    |       35.64        |    112.77   |         0.01        | WARNING: stale msgs |
+------------------------------------------+--------------------+--------------------+-------------+---------------------+---------------------+
Total Processing Time: 8.4 ms
```
showing each of the topics, and what the observed rates are. The jitter is the std dev of the rates over the chosen window. Time since last is useful if you want to check that you havent lost a specific topic. 


There is also a `topic_rate_monitor_interfaces` package, which defines a custom ros2 message type that you can use to subscribe to the data as well. 
The messages will be published on the topic `topic_rate_monitor/topic_rates` by default.
