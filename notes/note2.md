```markup
ros@sigyn7900:~/sigyn_ws$ ros2 run tf2_ros tf2_echo scan base_link
[INFO] [1731118087.408664501] [tf2_echo]: Waiting for transform scan ->  base_link: Invalid frame ID "scan" passed to canTransform argument target_frame - frame does not exist
At time 0.0
- Translation: [-0.000, -0.155, -0.305]
- Rotation: in Quaternion [-0.000, -0.000, 0.707, 0.707]
- Rotation: in RPY (radian) [-0.000, -0.000, 1.571]
- Rotation: in RPY (degree) [-0.000, -0.000, 90.000]
- Matrix:
  0.000 -1.000 -0.000 -0.000
  1.000  0.000  0.000 -0.155
  0.000 -0.000  1.000 -0.305
  0.000  0.000  0.000  1.000
```