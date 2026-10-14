#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

# الأبعاد الرياضية لروبوت TurtleBot3 Burger الافتراضي
r = 0.033  # نصف قطر العجلة بالمتر
d = 0.160  # المسافة بين العجلتين بالمتر

def joint_state_callback(msg):
    omega_L = msg.velocity[0]
    omega_R = msg.velocity[1]
    
    # حساب السرعة الخطية لكل عجلة (v = omega * r)
    v_L = omega_L * r
    v_R = omega_R * r
    
    # معادلة السرعة الخطية الكلية (Linear Velocity)
    v_linear = (v_R + v_L) / 2.0
    
    # معادلة السرعة الزاوية الكلية (Angular Velocity)
    w_angular = (v_R - v_L) / d
    
    rospy.loginfo("========================================")
    rospy.loginfo(f"Calculated Linear Velocity (v) : {v_linear:.4f} m/s")
    rospy.loginfo(f"Calculated Angular Velocity (w): {w_angular:.4f} rad/s")

def main():
    rospy.init_node('odom_speed_calculator_node', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
