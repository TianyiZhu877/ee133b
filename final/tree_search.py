import numpy as np

OPTIMAL = 0
LEFT = 1
RIGHT = 2


class treeSearch:
    def __init__(self, traj, model, visualizer, config):
        self.model = model
        self.visualizer = visualizer
        self.traj = traj
        self.config = config
    
        self.expansion_grid = []
        self.frontier_idx = 0
        self.frontier_node = None

        self.visualize_max_speed = 20
        self.visualize = False
        self.goal_idx = -1

        np.random.seed(0)
    
    def add_node(self, node):
        if node is None:
            # self.frontier_node = None
            return 
        
        if self.visualize:
            self.visualizer.plot_node(node, self.visualize_max_speed)

        idx, dist = node.get_idx_dist(self.traj)
        # frontier_expaned = False

        if (idx > self.frontier_idx):
            self.frontier_node = node
            self.frontier_idx = idx
        # else:
        #     self.frontier_node = None

    def grow_from_node(self, node, target_policy = OPTIMAL):
        current_node = node

        while not self.model.has_no_valid_children(current_node, self.traj):
            idx, dist = current_node.get_idx_dist(self.traj)

            if idx == self.goal_idx:
                return current_node.build_waypoints()
            
            d_lookahead = self.config['d_lookahead'] 
            idx = idx + int(d_lookahead / self.traj.dstep) # + np.random.uniform(0, 0)
            target, target_idx = self.traj.get_waypoint_bounded(idx)

            mean_action = self.model.controller(current_node, target, self.visualize_max_speed)
            action = self.generate_action(current_node, mean_action, target_idx)
            new_node = self.model.generate_child(current_node, action, self.traj)
            self.add_node(new_node)
            current_node = new_node


    def generate_action(self, node , mean_action, target_idx):
        acc, steer = mean_action
# simple clipping right now, adding randomization and gridding later
        max_steer = self.model.max_steering_angle(node.v_x)
        actual_acc = np.clip(acc, self.model.a_min, self.model.a_max)
        actual_steer = np.clip(steer, -max_steer, max_steer)
        # print('target steer, max_steer', steer, max_steer)
        return (actual_acc, actual_steer)

    def search(self, start, target_speed, goal_idx = -1):
        if (goal_idx<0) or (goal_idx>=len(self.traj.points)):
            goal_idx = len(self.traj.points) - 1
        self.goal_idx = goal_idx

        self.add_node(start)
        self.visualize_max_speed = target_speed

        if not start.is_valid(self.traj):
            print("Warning: start point is outside the track.")
            return None
        
        expansion_node = start
        # while True:
        return self.grow_from_node(expansion_node)
            # print('frontier node', self.frontier_node)
        




            
    
    

