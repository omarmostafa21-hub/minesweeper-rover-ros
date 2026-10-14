#!/usr/bin/env python3
# ============================================
# Velocity Calculator Node for Minesweeper Robot
# بيقرأ من /joint_states ويحسب ويبعت السرعات فقط على /odom_velocity
# ============================================

import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

# ============================================
# ثوابت الـ TurtleBot3 Burger
# ============================================
WHEEL_RADIUS = 0.033   # متر
WHEEL_BASE   = 0.160   # متر

# ============================================
class VelocityNode:
    def __init__(self):
        rospy.init_node('velocity_node', anonymous=False)
        rospy.loginfo("Velocity Calculator Node Started!")

        # آخر position للعجلات وحساب الوقت
        self.last_right = None
        self.last_left  = None
        self.last_time  = None

        # Publisher لتوبيك السرعة فقط
        self.odom_pub = rospy.Publisher('/odom_velocity', Odometry, queue_size=10)

        # Subscriber على /joint_states
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        rospy.loginfo("Subscribed to /joint_states")

    # ============================================
    def joint_states_callback(self, msg):
        """
        بتتشغل أوتوماتيك كل ما يجي message جديدة
        """
        # استخرج الـ position من المساج بالاسم لضمان الدقة
        try:
            right_idx = msg.name.index('wheel_right_joint')
            left_idx  = msg.name.index('wheel_left_joint')
            
            # شرط أمان: تأكد إن الـ position فيها داتا ومغطية الـ indexes
            if len(msg.position) <= max(right_idx, left_idx):
                return
        except ValueError:
            rospy.logwarn("wheel joints not found!")
            return

        current_right = msg.position[right_idx]  # بالـ radians
        current_left  = msg.position[left_idx]   # بالـ radians
        current_time  = msg.header.stamp

        # أول مرة — احفظ القيم بس ومتحسبش
        if self.last_right is None:
            self.last_right = current_right
            self.last_left  = current_left
            self.last_time  = current_time
            return

        # احسب الـ dt (الفرق الزمـني)
        dt = (current_time - self.last_time).to_sec()
        if dt <= 0:
            return

        # احسب الفرق في الـ radians (التغير في زاوية دوران العجلة)
        delta_right = current_right - self.last_right
        delta_left  = current_left  - self.last_left

        # حول من radians لمتر (المسافة المقطوعة لكل عجلة)
        dist_right  = delta_right * WHEEL_RADIUS
        dist_left   = delta_left  * WHEEL_RADIUS

        # احسب الحركة الكلية والـ delta_theta اللحظية
        dist_center = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / WHEEL_BASE

        # حساب السرعات اللحظية بالمعدلات الرياضية (السرعة = المسافة / الزمن)
        linear_x  = dist_center / dt
        angular_z = delta_theta / dt

        # ابعت رسالة الأودومتري بالسرعات فقط
        self.publish_velocity(linear_x, angular_z, current_time)

        # حدث الـ last values للدورة الجاية
        self.last_right = current_right
        self.last_left  = current_left
        self.last_time  = current_time

        # Log السرعات كل ثانية في الترمينال عشان تتابعها
        rospy.loginfo_throttle(1.0,
            f"Current Velocities -> Linear v={linear_x:.3f} m/s | Angular ω={angular_z:.3f} rad/s"
        )

    # ============================================
    def publish_velocity(self, linear_x, angular_z, current_time):
        # بنعمل إنبثاق لرسالة Odometry بس بنملا فيها جزء الـ twist (السرعات) بس
        odom_msg = Odometry()
        odom_msg.header.stamp     = current_time
        odom_msg.header.frame_id  = "odom"
        odom_msg.child_frame_id   = "base_link"
        
        # الـ Pose (الموقع) هيفضل أصفار لأننا مش بنحسبه
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.orientation.w = 1.0  # Identity quaternion
        
        # وضع السرعات المحسوبة بالمعادلات في مكانها الصح
        odom_msg.twist.twist.linear.x  = linear_x
        odom_msg.twist.twist.angular.z = angular_z

        self.odom_pub.publish(odom_msg)

# ============================================
if __name__ == '__main__':
    try:
        node = VelocityNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass