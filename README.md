This package(ump_suite) currently creates a GUI that enables both manual control and data acquisition from the robotic system.

For an external algorithm (learning, control, training, etc.) to work with this ROS2 package, knowledge on the following ROS topics are required:

Your code needs to...

Subscribe to:

/camera/image/compressed : sensor_msgs/msg/CompressedImage

/motor/live_counts: std_msgs/msg/Int32

/ump/live: std_msgs/msg/Int32MultiArray

Publish to:

/ump/target: std_msgs/msg/Int32MultiArray

/motor/target_counts: std_msgs/msg/Int32


These are the same topics that the GUI also communicates with for control and acquisition purposes.

Here is the RQT Graph at its current state.

<img width="2370" height="721" alt="rosgraph" src="https://github.com/user-attachments/assets/687d30ca-29bd-4bc9-99fa-ca9db4d94597" />
