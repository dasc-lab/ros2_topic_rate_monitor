# ROS2 Topic Rate Monitor

A small and simple utility to monitor the rate at which specific ros topics are being published. 

After the packages have been built, you can run the node, and specify the "config_file" ros2 parameter to point to the location of the config file. 

In the config file, you specify the topic name, the topic type, the expected rate, and optionally the tolerance (defaults to 10%), eg:
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
```

It will print a table like 
```
[INFO] [1737236388.099415105] [topic_rate_monitor]: 
+------------------------------------------+--------------------+--------------------+---------+
|                Topic Name                | Expected Rate (Hz) | Observed Rate (Hz) |  Status |
+------------------------------------------+--------------------+--------------------+---------+
|        /camera/infra1/camera_info        |    15.00 +- 10%    |        N/A         | WARNING |
| /visual_slam/tracking/vo_pose_covariance |    30.00 +- 10%    |       32.83        |    OK   |
+------------------------------------------+--------------------+--------------------+---------+
Total Processing Time: 0.098 ms
```
showing each of the topics, and what the observed rates are.


There is also a `topic_rate_monitor_interfaces` package, which defines a custom ros2 message type that you can use to subscribe to the data directly. 
The messages will be published on the topic `topic_rate_monitor/topic_rates` by default.
