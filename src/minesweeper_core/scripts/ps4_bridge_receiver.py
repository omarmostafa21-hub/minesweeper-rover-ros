#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import socket
import json

def run_receiver():
    rospy.init_node('ps4_bridge_receiver', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_sock.bind(("0.0.0.0", 9999))
    server_sock.settimeout(0.5)

    rospy.loginfo("🎮 PS4 Bridge Receiver listening on port 9999...")

    while not rospy.is_shutdown():
        try:
            data, _ = server_sock.recvfrom(1024)
            cmd = json.loads(data.decode())

            twist = Twist()
            twist.linear.x = cmd.get("linear", 0.0)
            twist.angular.z = cmd.get("angular", 0.0)

            pub.publish(twist)
        except socket.timeout:
            # لو الإشارة انقطعت يوقف الروبوت أوتوماتيكياً للأمان
            twist = Twist()
            pub.publish(twist)
        except Exception as e:
            rospy.logwarn(f"Error: {e}")

if __name__ == '__main__':
    try:
        run_receiver()
    except rospy.ROSInterruptException:
        pass
