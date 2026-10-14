#!/usr/bin/env python3
import rospy, math, os
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class GridMap:
    def __init__(self):
        rospy.init_node('grid_map_node')
        self.cols = [chr(i) for i in range(ord('A'), ord('S') + 1)]
        self.mines, self.cx, self.cy = {}, 0.0, 0.0
        rospy.Subscriber('/odom', Odometry, self.o_cb)
        rospy.Subscriber('/mine_status', Int8, self.m_cb)
        rospy.on_shutdown(self.save)

    def o_cb(self, m):
        self.cx, self.cy = m.pose.pose.position.x, m.pose.pose.position.y

    def m_cb(self, m):
        if m.data in [1, 2] and self.cx >= 0.5 and self.cy >= 0.5:
            ci = int(math.floor(self.cx - 0.5))
            ri = int(math.floor(self.cy - 0.5)) + 1
            if 0 <= ci < 19 and 1 <= ri <= 19 and (self.cols[ci], ri) not in self.mines:
                self.mines[(self.cols[ci], ri)] = m.data

    def save(self):
        p = os.path.expanduser('~/minesweeper_ws/src/minesweeper_core/maps')
        os.makedirs(p, exist_ok=True)
        with open(os.path.join(p, 'mines_report.txt'), 'w') as f:
            for (c, r), t in sorted(self.mines.items()):
                f.write(f"[{c}-{r}] : {'Surface' if t==1 else 'Buried'}\n")
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_xlim(0, 19); ax.set_ylim(0, 19)
        ax.set_xticks(range(20)); ax.set_yticks(range(20))
        ax.set_xticklabels([''] + self.cols); ax.set_yticklabels([''] + [str(i) for i in range(1, 20)])
        ax.grid(True)
        for (c, r), t in self.mines.items():
            ci, ri = self.cols.index(c), r - 1
            color = '#0D828A' if t == 1 else '#11222B'
            ax.add_patch(patches.Rectangle((ci, ri), 1, 1, facecolor=color))
            ax.text(ci+0.5, ri+0.5, 'S' if t==1 else 'B', color='white', ha='center', va='center', weight='bold')
        ax.invert_yaxis()
        plt.savefig(os.path.join(p, 'official_map.png'), bbox_inches='tight')
        plt.close()

if __name__ == '__main__':
    GridMap()
    rospy.spin()
