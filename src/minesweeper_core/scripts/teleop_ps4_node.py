#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

class Teleop:
    def __init__(self):
        rospy.init_node('teleop_ps4_node')
        self.p_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.p_arm = rospy.Publisher('/arm_control', Int8, queue_size=10)
        self.last_arm = None
        rospy.Subscriber('/joy', Joy, self.j_cb)

    def j_cb(self, m):
        t = Twist()
        dy = m.axes[7] if len(m.axes) > 7 else 0.0
        dx = m.axes[6] if len(m.axes) > 6 else 0.0
        if abs(dy) > 0.1 or abs(dx) > 0.1:
            t.linear.x, t.angular.z = dy * 0.12, dx * 0.35
        else:
            t.linear.x, t.angular.z = m.axes[1] * 0.55, m.axes[3] * 1.10
        self.p_cmd.publish(t)
        cmd = 0
        if len(m.buttons) > 5 and m.buttons[5]: cmd = 1
        elif len(m.buttons) > 4 and m.buttons[4]: cmd = 2
        elif len(m.axes) > 5 and m.axes[5] < -0.5: cmd = 3
        elif len(m.axes) > 2 and m.axes[2] < -0.5: cmd = 4
        if cmd != self.last_arm:
            self.last_arm = cmd
            self.p_arm.publish(Int8(data=cmd))

if __name__ == '__main__':
    Teleop()
    rospy.spin()
