#!/usr/bin/env python3
# basic form of ros2_control diff_drive_controller (borrowed from @rbonghi Nanosaur)
# https://github.com/rnanosaur/nanosaur_robot/blob/master/nanosaur_base/nanosaur_base/nanosaur.py
import signal
import rclpy
import math

from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32


class SkidSteer(None):
    """
    Node that transforms Twist messages on /cmd_vel
    to left/right motor controls (like a basic version
    of ros2_control's diff_drive_controller for Python)
    """
    def __init__(self):
        super().__init__('skid_steer')
        
        self.p = [0.0, 0.0] # [right, left]
        self.r = [0.0, 0.0] # [right, left]
        
        qos_profile = QoSProfile(depth=10)
        
        self.motor_pub = dict(
            right = self.create_publisher(Float32, 'motor_right', qos_profile),
            left = self.create_publisher(Float32, 'motor_left', qos_profile),
        )
        
        self.twist_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.on_twist,
            10)
        
        self.declare_parameter('wheel_diameter', 255.0)   # diameter of each wheel (in mm)
        self.declare_parameter('wheel_separation', 380.0) # distance between L/R wheels (in mm)

        self.declare_parameter('joint_states_rate', 5)
        self.joint_state = JointState()
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.timer_period = 1.0 / float(self.get_parameter('joint_states_rate').value)
        self.timer = self.create_timer(self.timer_period, self.transform_states)

    def on_twist(self, msg):
        # Store linear velocity and angular velocity
        v = msg.linear.x
        w = msg.angular.z
        
        # Convert linear and angular velocity to radiant motor speed
        self.get_logger().debug(f"v={v} w={w}")
        r = self.convert_speed(v, w)
        self.get_logger().debug(f"rad {r}")
        
        max_speed = 100 #self.rpm / 60.
        
        # Constrain between -max_speed << speed << max_speed.
        self.r = [
            max(-max_speed, min(max_speed, r[0])), 
            max(-max_speed, min(max_speed, r[1]))
        ]
        
        # Send a warning message if speed is over 
        if r[0] != self.r[0]:
            self.get_logger().warning(f"ref speed over {r[0] - self.r[0]}")
            
        if r[1] != self.r[1]:
            self.get_logger().warning(f"ref speed over {r[1] - self.r[1]}")
         
        msg = dict(
            right = Float32(),
            left = Float32(),
        )
        
        for i,m in enumerate(msg.items()):
            m[1].data = r[i]
            self.motor_pub[m[0]].publish(m[1])    
 
    def convert_speed(self, v, w):
        wheel_radius = float(self.get_parameter('wheel_radius').value) * 0.5
        wheel_separation = float(self.get_parameter('wheel_separation').value) * 0.5
        
        vr = v + wheel_separation * w
        vl = v - wheel_separation * w
        rr = vr / wheel_radius
        rl = vl / wheel_radius
        
        return [rr, rl]
        
    def transform_states(self):
        now = self.get_clock().now()
        
        # send the joint state and transform
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = list(self.motor_pub.keys())
        
        # Estimate angle position
        self.p[0] = (self.p[0] + self.r[0] * self.timer_period) % (2 * math.pi)
        self.p[1] = (self.p[1] + self.r[1] * self.timer_period) % (2 * math.pi)
        
        self.joint_state.position = self.p
        self.joint_state.velocity = self.r
        
        self.joint_pub.publish(self.joint_state)
    
    
def exit_signal(signum, frame):
    print(f"Close service {__name__} by signal {signum}".format(signum=signum))
    quit()


def main(args=None):
    rclpy.init(args=args)
    # Catch SIGTERM signal
    signal.signal(signal.SIGTERM, exit_signal)
    # Start Nanosaur
    nanosaur = NanoSaur()
    try:
        rclpy.spin(nanosaur)
    except (KeyboardInterrupt, SystemExit):
        pass
    # Destroy the node explicitly
    nanosaur.destroy_node()
    rclpy.shutdown()
    print(f"Node {__name__} is shutdown, quitting process...")


if __name__ == '__main__':
    main()
          
