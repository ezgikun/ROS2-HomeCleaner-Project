#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

MAX_SPEED = 0.6    
TURN_SPEED = 1.0 

msg = """
---------------------------
WASD KONTROL PANELI
---------------------------
   W
 A S D    SPACE: DUR

W: Ileri
S: Geri
A: Sola Don
D: Saga Don
SPACE: Hemen Dur

Q: Cikis
---------------------------
"""

class WASDTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish_velocity(self, linear, angular):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.publisher_.publish(twist)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = WASDTeleop()

    target_linear_vel = 0.0
    target_angular_vel = 0.0

    print(msg)

    try:
        while True:
            key = getKey()
            
            if key == 'w':
                target_linear_vel = MAX_SPEED
                target_angular_vel = 0.0
            elif key == 's':
                target_linear_vel = -MAX_SPEED
                target_angular_vel = 0.0
            elif key == 'a':
                target_linear_vel = 0.0
                target_angular_vel = TURN_SPEED
            elif key == 'd':
                target_linear_vel = 0.0
                target_angular_vel = -TURN_SPEED
            elif key == ' ': 
                target_linear_vel = 0.0
                target_angular_vel = 0.0
            elif key == 'q':
                break
            else:
                target_linear_vel = 0.0
                target_angular_vel = 0.0

            node.publish_velocity(target_linear_vel, target_angular_vel)

    except Exception as e:
        print(e)

    finally:
        node.publish_velocity(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
