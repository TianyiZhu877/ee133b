import matplotlib.pyplot as plt
import time
import numpy as np  

class Visualizer:
    def __init__(self, update_rate=0.1):
        # plt.ion()
        plt.show()
        plt.axis('equal')


    def plot_trajectory(self, traj):
        # plt.show()

        tan_x = traj.tangent[:, :1]
        tan_y = traj.tangent[:, 1:]

        right_boundary = traj.points + np.hstack((tan_y, -tan_x))*traj.widths[:, np.newaxis]
        left_boundary = traj.points + np.hstack((-tan_y, tan_x))*traj.widths[:, np.newaxis]
        plt.plot(right_boundary[:, 0], right_boundary[:, 1], '-', color='grey')
        plt.plot(left_boundary[:, 0], left_boundary[:, 1], '-', color='grey')
        # Plot the smoothed line
        # plt.plot(smoothed_line[:, 0], smoothed_line[:, 1], 'r-', label='Smoothed Line', linewidth=2)
        # print(traj.widths)

        # for i in range(traj.points.shape[0]):
        #     # print((traj.points[i, 0], traj.points[i, 1]), traj.widths[i])
        #     # circle = plt.Circle((traj.points[i, 0], traj.points[i, 1]), traj.widths[i], color='grey')
        #     # plt.gca().add_patch(circle)
        #     plt.scatter(traj.points[i, 0], traj.points[i, 1], s=traj.widths[i], color='grey')

        plt.plot(traj.points[:, 0], traj.points[:, 1], 'y-') # , label='Original Line')
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

        self.show()

    def show(self):
        plt.pause(1e-3)
