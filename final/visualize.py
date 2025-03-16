import matplotlib.pyplot as plt
import time
import numpy as np  

class Visualizer:
    def __init__(self, update_rate=0.1):
        # plt.ion()
        # plt.show()
        plt.axis('equal')
        # pass
        self.last_update = time.time()

    def plot_waypoints(self, waypoints, max_t = 10):
        x = []
        y = []
        c = []
        for i in range(len(waypoints)-1,1,-1):
            speed_ratio = min(abs(waypoints[i].v_x)/max_t,1)
            # x.append(node.x)
            # y.append(node.y)
            # c.append([speed_ratio, 1-speed_ratio, 0])
        
            # plt.plot(x,y , '-', color=c)
            # print(node.x, node.y)
            # print((waypoints[i-1].x, waypoints[i].x), (waypoints[i-1].y, waypoints[i].y))
            plt.plot((waypoints[i-1].x, waypoints[i].x), (waypoints[i-1].y, waypoints[i].y) , '-', color=[1-speed_ratio, 1-speed_ratio, speed_ratio])
            # self.show()
        
        # plt.plot(x, y , '-')
        plt.show()

    def plot_trajectory(self, traj):
        # plt.show()

        plt.plot(traj.right_boundary[:, 0], traj.right_boundary[:, 1], '-', color='grey')
        plt.plot(traj.left_boundary[:, 0], traj.left_boundary[:, 1], '-', color='grey')
        # Plot the smoothed line
        # plt.plot(smoothed_line[:, 0], smoothed_line[:, 1], 'r-', label='Smoothed Line', linewidth=2)
        # print(traj.widths)

        # for i in range(traj.points.shape[0]):
        #     # print((traj.points[i, 0], traj.points[i, 1]), traj.widths[i])
        #     # circle = plt.Circle((traj.points[i, 0], traj.points[i, 1]), traj.widths[i], color='grey')
        #     # plt.gca().add_patch(circle)
        #     plt.scatter(traj.points[i, 0], traj.points[i, 1], s=traj.widths[i], color='grey')

        # plt.plot(traj.points[:, 0], traj.points[:, 1], 'y-') # , label='Original Line')
        plt.plot(traj.estim_path[:, 0], traj.estim_path[:, 1], 'y-') # , label='Original Line')
        # self.show()

    
    def plot_node(self, node, max_t = 10):

        if node.parent is not None:
            x = [node.parent.x, node.x]
            y= [node.parent.y, node.y]
        else:
            x = node.x
            y= node.y
        
        # plt.plot(x,y , '-', color='green')
        speed_ratio = min(abs(node.v_x)/max_t,1)
        plt.plot(x,y , '-', color=[speed_ratio, 1-speed_ratio, 0])

        current_time = time.time()
        if current_time - self.last_update > 1e-2:
            self.last_update = time.time()
            self.show()

    def show(self):
        plt.pause(1e-3)
