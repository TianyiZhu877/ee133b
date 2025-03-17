import matplotlib.pyplot as plt
import time
import numpy as np 
from utils import R, angle_between_vectors

class Visualizer:
    def __init__(self, update_rate=0.1):
        # plt.ion()
        # plt.show()
        plt.axis('equal')
        # pass
        self.last_update = time.time()
        self.interval = 1e3
        self.countdown = self.interval

        self.model = None
        self.chassy = np.array([(1,0.2), (0.6, 0.25),(0.5,0.45), (0, 0.6)])

        self.x_lim = (-5, 5)
        self.y_lim = (-2, 16)

        self.wheels = np.array([(-0.08, 0), (0.08, 0)])
        self.accelerator_bar = np.array([(-self.x_lim[0]/1.8, 0), (0, 0)])
        self.accelerator_seperation = 0.6
        self.accelerator_levels = 6

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
        if current_time - self.last_update > 0.1:
            self.last_update = time.time()
        # self.countdown -= 1
        # if self.countdown <= 0:
        #     self.countdown = self.interval
            self.show()
    

    def transform(self, points_list, theta = 0, dx = 0, dy = 0):
        new_points_list = []
        for points in points_list:
            # print(points)
            new_points = points @ R(theta)
            new_points[:,0] += dx
            new_points[:,1] += dy
            new_points_list.append(new_points)
        return new_points_list
    

    def animation(self, model, waypoints, traj):
        start = True
        i = -100
        # for node in waypoints[::-2]:
        while True:
            idx = np.clip(i*2, 0, len(waypoints))
            if idx==len(waypoints):
                node = model.generate_child(node, (0,0), traj, True)
            else:
                node = waypoints[-idx-1]
            i+=1
            segments = []
            colors = []
            widths = []

            acc, steer = 0, 0
            if node.last_action is not None:
                acc, steer = node.last_action


            # draw the wheels
            segments += [np.array([[model.L/2, model.L_f/2], [model.L/2, -model.L_f/2]]), 
                         np.array([[-model.L/2, model.L_f/2], [-model.L/2, -model.L_f/2]])]
            colors+=['grey', 'grey']
            widths+=[3,3]

            front_left_wheel = self.transform([self.wheels], steer, model.L/2, model.L_f/2)[0]
            front_right_wheel = self.transform([self.wheels], steer, model.L/2, -model.L_f/2)[0]
            rear_left_wheel = self.transform([self.wheels], 0, -model.L/2, model.L_f/2)[0]
            rear_right_wheel = self.transform([self.wheels], 0, -model.L/2, -model.L_f/2)[0]
            segments += [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
            for a in range(4):
                colors.append('black')
                widths.append(4)

            # draw the car chassy
            car_size = np.array([model.L, model.L_f])
            chassy_reflected = self.chassy.copy()
            chassy_reflected[:,1] = -chassy_reflected[:,1]
            chassy = np.vstack((self.chassy,chassy_reflected[::-1]) )
            chassy[:,1] = chassy[:,1] / 1.5
            chassy[:,0] = chassy[:,0] - 0.5
            # chassy = self.chassy
            chassy = chassy*car_size 
            # print(chassy)
            # plt.plot(chassy[:,0], chassy[:,1])
            # plt.show()

            segments.append(chassy)
            colors.append('orange')
            widths.append(1)

            # the car's transform
            segments = self.transform(segments, node.theta, node.x, node.y)



            # draw track
            segments.append(traj.right_boundary)
            colors.append('grey')
            widths.append(1)

            segments.append(traj.left_boundary)
            colors.append('grey')
            widths.append(1)

                


            idx, dist = node.get_idx_dist(traj)
            
            dtheta = angle_between_vectors(np.array([0,1]), traj.tangent[idx])
            segments = self.transform(segments, 0, -traj.points[idx, 0], -traj.points[idx, 1])
            segments = self.transform(segments, dtheta)

            if i<0:
                plt.text(0, 7, 'Ready', fontsize=12)

            # draw accelerator
            y = 3
            color = 'green'
            max = model.a_max
            dy = self.accelerator_seperation
            if acc<0:
                dy = -dy
                y = dy+y
                color = 'red'
                # acc = -acc
                max = model.a_min
            
            # print(i, int(acc/max*self.accelerator_levels))
            for a in range(int(acc/max*self.accelerator_levels)):
                y += dy
                segments += self.transform([self.accelerator_bar], 0, self.x_lim[1], y)
                colors.append(color)
                widths.append(6)



            self.show_segments(segments, colors, widths, model.dt)

                # self.show()
                # current_time = time.time()
                # while time.time() - current_time < 3:
                #     pass
                # start = False
                




    def show_segments(self, segments, colors, widths, dt):
        while (time.time() - self.last_update < dt):
            print('waiting')
            pass
        plt.cla()
        plt.axis('equal')
        plt.xlim(self.x_lim)
        plt.ylim(self.y_lim)
        self.last_update = time.time()
        for i, segment in enumerate(segments):
            plt.plot(segment[:, 0], segment[:, 1] , '-', color = colors[i], linewidth=widths[i])
        self.show()



        
        

    def show(self):
        plt.pause(1e-3)
