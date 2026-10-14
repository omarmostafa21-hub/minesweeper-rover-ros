#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped

class ArenaGrid19x19:
    def __init__(self):
        rospy.init_node('arena_grid_19x19', anonymous=True)

        # إعدادات الشبكة 19x19
        self.grid_size = 19
        self.cell_size = rospy.get_param('~cell_size', 1.0) # حجم المربع بالمتر

        # تهيئة مصفوفة الخريطة: -1 يعني لم تُكتشف بعد (Unknown)
        self.map_data = [-1] * (self.grid_size * self.grid_size)

        # ناشر الخريطة والمسار
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10, latch=True)
        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=10)

        self.robot_path = Path()
        self.robot_path.header.frame_id = "odom"

        # إعداد رسالة الـ OccupancyGrid الثابتة
        self.grid_msg = OccupancyGrid()
        self.grid_msg.header.frame_id = "odom"
        self.grid_msg.info.resolution = self.cell_size
        self.grid_msg.info.width = self.grid_size
        self.grid_msg.info.height = self.grid_size
        # نقطة الأصل (0,0) بحيث يبدأ الروبوت من الزاوية أو المركز
        self.grid_msg.info.origin.position.x = 0.0
        self.grid_msg.info.origin.position.y = 0.0
        self.grid_msg.info.origin.orientation.w = 1.0

        # الاستماع لقراءات الأودومتري
        rospy.Subscriber('/odom', Odometry, self.odom_cb)

        # مؤقت لنشر الخريطة بمعدل 5Hz
        rospy.Timer(rospy.Duration(0.2), self.publish_map)
        rospy.loginfo("🗺️ 19x19 Grid Map Node Initialized Successfully.")

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # تحديث المسار التراكمي
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.robot_path.poses.append(pose_stamped)
        self.path_pub.publish(self.robot_path)

        # تحويل إحداثيات (x, y) بالمتر إلى مؤشرات المصفوفة (col, row)
        col = int(x / self.cell_size)
        row = int(y / self.cell_size)

        # التأكد أن الروبوت داخل حدود الحقل 19x19
        if 0 <= col < self.grid_size and 0 <= row < self.grid_size:
            index = row * self.grid_size + col
            # 0 يعني مساحة تم مسحها وخالية (Cleared / Free space)
            if self.map_data[index] != 0:
                self.map_data[index] = 0
                rospy.loginfo(f"📍 Rover Cleared Cell: Row {row+1}, Col {col+1}")

    def publish_map(self, event):
        self.grid_msg.header.stamp = rospy.Time.now()
        self.grid_msg.data = self.map_data
        self.map_pub.publish(self.grid_msg)

if __name__ == '__main__':
    try:
        node = ArenaGrid19x19()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
