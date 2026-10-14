#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Int32MultiArray, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
from tf.transformations import quaternion_from_euler

class WheelOdometry:
    def __init__(self):
        rospy.init_node('wheel_odometry', anonymous=True)

        self.wheel_track = rospy.get_param('~wheel_track', 0.52)
        self.wheel_diameter = rospy.get_param('~wheel_diameter', 0.10)
        self.ticks_per_rev = rospy.get_param('~ticks_per_rev', 456.0)

        # حساب المسافة لكل تكة بناءً على 456 تكة وقطر 10 سم
        self.meters_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        # البداية عند منتصف المربع A1 ومتجه لأعلى (90 درجة)
        self.x = 0.5
        self.y = 0.5
        self.base_yaw = math.pi / 2.0
        self.th = self.base_yaw

        self.has_imu = False
        self.imu_yaw_val = 0.0

        self.last_ticks = None
        self.last_time = rospy.Time.now()

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber('/wheel_ticks', Int32MultiArray, self.ticks_cb)
        rospy.Subscriber('/imu/yaw', Float32, self.imu_cb)
        rospy.loginfo("✅ Standard Hybrid Odometry Active (456 ticks/rev).")

    def imu_cb(self, msg):
        self.has_imu = True
        self.imu_yaw_val = msg.data
        # تحديث الزاوية مباشرة مع الجايرو زي النسخة الشغالة
        self.th = math.atan2(
            math.sin(self.base_yaw + self.imu_yaw_val),
            math.cos(self.base_yaw + self.imu_yaw_val)
        )

    def ticks_cb(self, msg):
        current_time = rospy.Time.now()
        ticks = msg.data
        if len(ticks) < 6:
            return

        if self.last_ticks is None:
            self.last_ticks = list(ticks)
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).to_sec()
        if dt <= 0:
            return

        delta_left_ticks = (
            (ticks[0] - self.last_ticks[0]) +
            (ticks[2] - self.last_ticks[2]) +
            (ticks[4] - self.last_ticks[4])
        ) / 3.0

        delta_right_ticks = (
            (ticks[1] - self.last_ticks[1]) +
            (ticks[3] - self.last_ticks[3]) +
            (ticks[5] - self.last_ticks[5])
        ) / 3.0

        self.last_ticks = list(ticks)
        self.last_time = current_time

        d_left = delta_left_ticks * self.meters_per_tick
        d_right = delta_right_ticks * self.meters_per_tick
        d_center = (d_left + d_right) / 2.0

        # لو الـ IMU مش شغال، يستخدم الإنكودرز
        if not self.has_imu:
            d_theta = (d_right - d_left) / self.wheel_track
            self.th += d_theta
            self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)

        vx = d_center / dt
        odom_quat = quaternion_from_euler(0, 0, self.th)

        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*odom_quat))
        odom.twist.twist = Twist(Vector3(vx, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        node = WheelOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
