"""Shared node entry point. Ctrl-C / launch shutdown ends the node with exit code 0 instead of a traceback
and exit code 1 (which ros2 launch reports as "process has died")."""
import rclpy
from rclpy.executors import ExternalShutdownException


def run(node_cls):
    rclpy.init()
    node = node_cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():      # a real failure; otherwise a timer publish raced the SIGINT shutdown (RCLError)
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
