#!/usr/bin/env python3
import rospy
import threading
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
import tf.transformations

class LiveGridMapGUI:
    def __init__(self):
        rospy.init_node('live_grid_map_gui', anonymous=True)
        
        # 19x19 Grid (0: Free, 1: Surface Mine, 2: Buried Mine)
        self.grid = np.zeros((19, 19), dtype=int)
        
        # Lock لمنع تعارض الـ Threads
        self.lock = threading.Lock()
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.path_x = []
        self.path_y = []
        
        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('/mine_status', Int8, self.mine_cb, queue_size=10)
        
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.manager.set_window_title('Minesweeper Live Arena (19x19)')
        
        self.col_labels = [chr(ord('A') + i) for i in range(19)]
        self.row_labels = [str(i + 1) for i in range(19)]
        
    def odom_cb(self, msg):
        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        _, _, yaw = tf.transformations.euler_from_quaternion(q)
        
        gx = msg.pose.pose.position.x - 0.5
        gy = msg.pose.pose.position.y - 0.5
        
        with self.lock:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            self.robot_yaw = yaw
            self.path_x.append(gx)
            self.path_y.append(gy)
            if len(self.path_x) > 3000:
                self.path_x.pop(0)
                self.path_y.pop(0)

    def mine_cb(self, msg):
        gx = int(np.floor(self.robot_x - 0.5))
        gy = int(np.floor(self.robot_y - 0.5))
        if 0 <= gx < 19 and 0 <= gy < 19:
            with self.lock:
                self.grid[gy, gx] = msg.data

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.lock:
                cur_x = self.robot_x
                cur_y = self.robot_y
                cur_yaw = self.robot_yaw
                pts_x = list(self.path_x)
                pts_y = list(self.path_y)
                grid_copy = np.copy(self.grid)
            
            # التأكد الصارم من تطابق أطوال مصفوفات المسار
            min_len = min(len(pts_x), len(pts_y))
            pts_x = pts_x[:min_len]
            pts_y = pts_y[:min_len]

            self.ax.clear()
            
            self.ax.set_xlim(-1, 19)
            self.ax.set_ylim(19, -1)
            
            self.ax.set_xticks(np.arange(19))
            self.ax.set_yticks(np.arange(19))
            self.ax.set_xticklabels(self.col_labels)
            self.ax.set_yticklabels(self.row_labels)
            self.ax.grid(True, color='gray', linestyle='--', linewidth=0.7)
            
            # رسم الألغام
            for r in range(19):
                for c in range(19):
                    val = grid_copy[r, c]
                    if val == 1:
                        self.ax.add_patch(plt.Rectangle((c - 0.5, r - 0.5), 1, 1, color='teal', alpha=0.6))
                        self.ax.text(c, r, 'S', color='white', ha='center', va='center', fontweight='bold', fontsize=11)
                    elif val == 2:
                        self.ax.add_patch(plt.Rectangle((c - 0.5, r - 0.5), 1, 1, color='navy', alpha=0.6))
                        self.ax.text(c, r, 'B', color='white', ha='center', va='center', fontweight='bold', fontsize=11)

            # رسم المسار
            if min_len > 1:
                self.ax.plot(pts_x, pts_y, color='blue', linewidth=1.8, alpha=0.8)

            # رسم الروبوت
            rx = cur_x - 0.5
            ry = cur_y - 0.5
            self.ax.plot(rx, ry, 'ro', markersize=8)
            
            dx = 0.6 * np.cos(cur_yaw)
            dy = -0.6 * np.sin(cur_yaw)
            self.ax.arrow(rx, ry, dx, dy, head_width=0.25, head_length=0.25, fc='red', ec='red')

            self.ax.set_title(f'Pose: X={cur_x:.2f}m, Y={cur_y:.2f}m, Yaw={np.degrees(cur_yaw):.1f}°', fontsize=10)
            
            self.fig.canvas.draw_idle()
            plt.pause(0.01)
            rate.sleep()

if __name__ == '__main__':
    try:
        LiveGridMapGUI().run()
    except rospy.ROSInterruptException:
        pass
