import matplotlib.pyplot as plt
import time

class Visualizer:
    def __init__(self):
        plt.axis('equal')


    def plot_trajectory(self, traj):
        # plt.show()

        # Plot the smoothed line
        # plt.plot(smoothed_line[:, 0], smoothed_line[:, 1], 'r-', label='Smoothed Line', linewidth=2)
        # print(traj.widths)
        for i in range(traj.points.shape[0]):
            # print((traj.points[i, 0], traj.points[i, 1]), traj.widths[i])
            circle = plt.Circle((traj.points[i, 0], traj.points[i, 1]), traj.widths[i], color='grey')
            plt.gca().add_patch(circle)

        plt.plot(traj.points[:, 0], traj.points[:, 1], 'y-', label='Original Line', alpha=0.2)
        self.show()

    
    def plot_node(self, node):

        if node.parent is not None:
            x = [node.parent.x, node.x]
            y= [node.parent.y, node.y]
        else:
            x = node.x
            y= node.y
        
        plt.plot(x,y , '-', color='green')

        self.show()

    def show(self):
        plt.draw()
