"""Shared node entry point. Ctrl-C / launch shutdown ends the node with exit code 0 instead of a traceback
and exit code 1 (which ros2 launch reports as "process has died")."""
import signal

import rclpy
from rclpy.executors import ExternalShutdownException


def run(node_cls):
    # Ctrl-C is left to rclpy: the SIGINT handler init() installs shuts the context down and spin() ends with
    # ExternalShutdownException. Python's KeyboardInterrupt is switched off first because it surfaced at random
    # points: in the cleanup when launch forwarded a second SIGINT, and inside rclpy's message conversion, where
    # it came out as a RuntimeError.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
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
        # Also replaces rclpy's handler: plain SIG_IGN outlives interpreter teardown, so a late SIGINT can't kill
        # the node with -2 while rclpy/DDS objects are being destroyed.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
