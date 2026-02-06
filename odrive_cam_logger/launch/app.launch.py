from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    # If PySpin needs a special venv like before, replace the camera Node with ExecuteProcess.
    # For now this assumes PySpin works in your normal environment.
    return LaunchDescription([
        Node(
            package="odrive_cam_logger",
            executable="odrive_driver_node",
            output="screen",
            parameters=[{
                "poll_ms": 50,
                "goto_speed_turns_s": 0.5,
                "deadband_counts": 200,
            }],
        ),

        Node(
            package="odrive_cam_logger",
            executable="camera_node",
            output="screen",
            parameters=[{
                "publish_hz": 30.0,
                "record_fps": 20.0,
                "jpeg_quality": 80,
            }],
        ),

        Node(
            package="odrive_cam_logger",
            executable="logger_node",
            output="screen",
            parameters=[{
                "log_interval_ms": 500,
            }],
        ),
        
        Node(
            package="odrive_cam_logger",
            executable="gui_node",
            output="screen",
        ),
    ])
