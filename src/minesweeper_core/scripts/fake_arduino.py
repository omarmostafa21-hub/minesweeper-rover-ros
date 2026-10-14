#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Int8
from sensor_msgs.msg import Imu

class FakeArduino:
    def __init__(self):
        rospy.init_node('fake_arduino_node')
        self.pub_ticks = rospy.Publisher('/wheel_ticks', Int32MultiArray, queue_size=10)
        self.pub_imu = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.pub_mine = rospy.Publisher('/mine_status', Int8, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb)
        
        self.ticks = [0, 0, 0, 0, 0, 0]
        self.vx = 0.0
        self.wz = 0.0

    def cmd_cb(self, msg):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def run(self):
        rate = rospy.Rate(20) # 20 Hz
        while not rospy.is_shutdown():
            dt = 0.05
            # حساب نبضات تقريبية
            delta_ticks_r = int((self.vx + self.wz * 0.20) * 3183 * dt)
            delta_ticks_l = int((self.vx - self.wz * 0.20) * 3183 * dt)
            
            for i in range(3): self.ticks[i] += delta_ticks_r
            for i in range(3, 6): self.ticks[i] += delta_ticks_l
            
            tm = Int32MultiArray(data=self.ticks)
            self.pub_ticks.publish(tm)
            
            imu = Imu()
            imu.angular_velocity.z = self.wz
            self.pub_imu.publish(imu)
            rate.sleep()

if __name__ == '__main__':
    FakeArduino().run()
