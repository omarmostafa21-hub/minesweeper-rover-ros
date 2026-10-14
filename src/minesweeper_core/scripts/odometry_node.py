#!/usr/bin/env python3
import rospy, math, tf
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion

class Odom:
    def __init__(self):
        rospy.init_node('odometry_node')
        self.r, self.b, self.cpr = 0.05, 0.40, 1000.0
        self.dp = (2.0 * math.pi * self.r) / self.cpr
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.lr, self.ll, self.init = 0.0, 0.0, False
        self.gz, self.lt = 0.0, rospy.Time.now()
        self.path = Path()
        self.path.header.frame_id = "odom"
        rospy.Subscriber('/wheel_ticks', Int32MultiArray, self.t_cb)
        rospy.Subscriber('/imu/data', Imu, self.i_cb)
        self.p_odom = rospy.Publisher('/odom', Odometry, queue_size=20)
        self.p_path = rospy.Publisher('/robot_path', Path, queue_size=20)
        self.br = tf.TransformBroadcaster()

    def i_cb(self, m): 
        self.gz = m.angular_velocity.z

    def t_cb(self, m):
        t = rospy.Time.now()
        dt = (t - self.lt).to_sec()
        if dt <= 0: return
        rt = (m.data[0] + m.data[1] + m.data[2]) / 3.0
        lt = (m.data[3] + m.data[4] + m.data[5]) / 3.0
        if not self.init:
            self.lr, self.ll, self.init, self.lt = rt, lt, True, t
            return
        dr = (rt - self.lr) * self.dp
        dl = (lt - self.ll) * self.dp
        self.lr, self.ll = rt, lt
        d = (dr + dl) / 2.0
        dth = self.gz * dt if abs(self.gz) > 0.015 else (dr - dl) / self.b
        self.x += d * math.cos(self.th + dth / 2.0)
        self.y += d * math.sin(self.th + dth / 2.0)
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))
        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.br.sendTransform((self.x, self.y, 0), q, t, "base_link", "odom")
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = t, "odom", "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation = Quaternion(*q)
        self.p_odom.publish(o)
        ps = PoseStamped()
        ps.header.stamp, ps.header.frame_id = t, "odom"
        ps.pose.position.x, ps.pose.position.y = self.x, self.y
        self.path.poses.append(ps)
        self.p_path.publish(self.path)
        self.lt = t

if __name__ == '__main__':
    Odom()
    rospy.spin()
