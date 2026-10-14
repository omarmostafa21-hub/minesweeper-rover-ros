#!/usr/bin/env python3
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

class MinesweeperArenaGUI:
    def __init__(self):
        rospy.init_node('minesweeper_arena_gui', anonymous=True)

        self.grid_size = 19
        self.cell_size = rospy.get_param('~cell_size', 1.0)

        # البداية عند 0.5 و 0.5
        self.x = 0.5
        self.y = 0.5
        self.yaw = math.pi / 2.0

        self.path_x = [self.x]
        self.path_y = [self.y]

        self.visited_cells = {}
        self.visited_cells[(0, 0)] = ('S', '#5dade2')

        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/mine_status', String, self.mine_cb)

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.manager.set_window_title('Minesweeper Live Arena (19x19)')

        self.cols_labels = [chr(ord('A') + i) for i in range(19)]
        self.rows_labels = [str(i + 1) for i in range(19)]

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        (_, _, self.yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

        self.path_x.append(self.x)
        self.path_y.append(self.y)

        col = int(self.x / self.cell_size)
        row = int(self.y / self.cell_size)

        if 0 <= col < self.grid_size and 0 <= row < self.grid_size:
            if (col, row) not in self.visited_cells:
                self.visited_cells[(col, row)] = ('', '#85c1e9')

    def mine_cb(self, msg):
        col = int(self.x / self.cell_size)
        row = int(self.y / self.cell_size)
        if 0 <= col < self.grid_size and 0 <= row < self.grid_size:
            self.visited_cells[(col, row)] = ('MINE', '#e74c3c')

    def update(self, frame):
        self.ax.clear()
        # محاور تبدأ من 0 إلى 19 ليكون الصف 1 في القاع
        self.ax.set_xlim(0, self.grid_size * self.cell_size)
        self.ax.set_ylim(0, self.grid_size * self.cell_size)

        centers = [(i + 0.5) * self.cell_size for i in range(self.grid_size)]
        ticks = [i * self.cell_size for i in range(self.grid_size + 1)]
        self.ax.set_xticks(centers)
        self.ax.set_xticklabels(self.cols_labels, fontsize=8)
        self.ax.set_yticks(centers)
        self.ax.set_yticklabels(self.rows_labels, fontsize=8)
        self.ax.set_xticks(ticks, minor=True)
        self.ax.set_yticks(ticks, minor=True)
        self.ax.grid(which='minor', color='#bdc3c7', linestyle='-', linewidth=0.8)
        self.ax.tick_params(which='both', length=0)

        for (c, r), (label, color) in self.visited_cells.items():
            rect = patches.Rectangle(
                (c * self.cell_size, r * self.cell_size),
                self.cell_size, self.cell_size,
                linewidth=0, facecolor=color, alpha=0.75
            )
            self.ax.add_patch(rect)
            if label:
                self.ax.text(
                    (c + 0.5) * self.cell_size, (r + 0.5) * self.cell_size, label,
                    color='white', weight='bold', fontsize=9,
                    ha='center', va='center'
                )

        if len(self.path_x) > 1:
            self.ax.plot(self.path_x, self.path_y, color='#2980b9', linewidth=1.8, alpha=0.85)

        self.ax.plot(self.x, self.y, 'ro', markersize=6)
        dx = 0.4 * self.cell_size * math.cos(self.yaw)
        dy = 0.4 * self.cell_size * math.sin(self.yaw)
        self.ax.arrow(self.x, self.y, dx, dy, head_width=0.15 * self.cell_size, head_length=0.15 * self.cell_size, fc='red', ec='red')

        yaw_deg = (math.degrees(self.yaw) + 360) % 360
        self.ax.set_title(f"Pose: X={self.x:.2f}m, Y={self.y:.2f}m, Yaw={yaw_deg:.1f}°", fontsize=10, pad=10)

if __name__ == '__main__':
    gui = MinesweeperArenaGUI()
    ani = FuncAnimation(gui.fig, gui.update, interval=100)
    plt.tight_layout()
    plt.show()
