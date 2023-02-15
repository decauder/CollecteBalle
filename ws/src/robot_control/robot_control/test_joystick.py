import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from inputs import get_gamepad
from inputs import devices

for device in devices:
    print(device)
