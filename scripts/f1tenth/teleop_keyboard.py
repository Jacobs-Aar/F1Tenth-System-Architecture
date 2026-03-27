#!/usr/bin/env python3
"""
scripts/f1tenth/teleop_keyboard.py
====================================
Minimal keyboard teleoperation node for testing the VESC without the
full perception stack. Publishes AckermannDriveStamped to /drive.

Run INSIDE the Docker container:
  python3 /ros2_ws/src/../scripts/f1tenth/teleop_keyboard.py

Or copy it into the container and run:
  python3 teleop_keyboard.py

Controls:
  W / UP    — increase speed
  S / DOWN  — decrease speed
  A / LEFT  — steer left
  D / RIGHT — steer right
  SPACE     — emergency stop (zero speed + zero steering)
  Q         — quit
"""

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


SPEED_STEP = 0.1       # m/s per key press
STEER_STEP = 0.05      # radians per key press
MAX_SPEED = 3.0
MAX_STEER = 0.34


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.speed = 0.0
        self.steer = 0.0
        self.get_logger().info(
            'Teleop ready. W/S=speed, A/D=steer, SPACE=stop, Q=quit')

    def publish_drive(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steer
        self.pub.publish(msg)
        self.get_logger().info(
            f'speed={self.speed:.2f} m/s  steer={self.steer:.2f} rad')


def get_key():
    """Read a single keypress from stdin (blocking)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # Handle arrow keys (escape sequences)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            if ch2 == '[':
                return {'A': 'w', 'B': 's', 'C': 'd', 'D': 'a'}.get(ch3, '')
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = TeleopKeyboard()

    print('\n--- F1Tenth Keyboard Teleop ---')
    print('W/S = speed up/down')
    print('A/D = steer left/right')
    print('SPACE = stop')
    print('Q = quit\n')

    try:
        while True:
            key = get_key().lower()
            if key == 'q':
                node.speed = 0.0
                node.steer = 0.0
                node.publish_drive()
                break
            elif key == 'w':
                node.speed = min(node.speed + SPEED_STEP, MAX_SPEED)
            elif key == 's':
                node.speed = max(node.speed - SPEED_STEP, -MAX_SPEED)
            elif key == 'a':
                node.steer = min(node.steer + STEER_STEP, MAX_STEER)
            elif key == 'd':
                node.steer = max(node.steer - STEER_STEP, -MAX_STEER)
            elif key == ' ':
                node.speed = 0.0
                node.steer = 0.0
            node.publish_drive()
    except KeyboardInterrupt:
        pass
    finally:
        node.speed = 0.0
        node.steer = 0.0
        node.publish_drive()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
