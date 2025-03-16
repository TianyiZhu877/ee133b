import numpy as np
from scipy.stats import truncnorm

ESTIM = 0
LEFT = 1
RIGHT = 2


class treeSearch:
    def __init__(self, traj, model, visualizer, config):
        self.model = model
        self.visualizer = visualizer
        self.traj = traj
        self.config = config
    
        self.node_grid = []
        for i in range(self.config['prev_horizon']):
            self.node_grid.append([])

        self.frontier_idx = 0

        self.visualize_max_speed = 25
        self.visualize = False
        self.goal_idx = -1

        self.prev_frontier = 0
        self.mean_rollback =  self.config['mean_rollback'] 

        # np.random.seed(0)
        self.rng = np.random.default_rng(0)

    
    def add_node(self, node):
        if node is None:
            # self.frontier_node = None
            return 
        
        if self.visualize:
            self.visualizer.plot_node(node, self.visualize_max_speed)

        idx, dist = node.get_idx_dist(self.traj)
        # frontier_expaned = False

        if (idx > self.frontier_idx):
            # self.frontier_node = node
            new_idx = idx-self.frontier_idx
            if new_idx > self.config['prev_horizon']:
                print('Search tree warning: frontier pushed by two much')


            self.node_grid = self.node_grid[:-new_idx]
            for i in range(new_idx):
                self.node_grid.insert(0, [])

            self.frontier_idx = idx
        
        grid_x = self.frontier_idx - idx
        # print(grid_x, grid_y)
        # self.grid[grid_x, grid_y] += 1
        self.node_grid[grid_x].append(node)
        # else:
        #     self.frontier_node = None

    def grow_from_node(self, node, policy = ESTIM):
        current_node = node

        num_growed = 1
        num_understeer = 1

        while not self.model.has_no_valid_children(current_node, self.traj):
            original_idx, dist = current_node.get_idx_dist(self.traj)

            if original_idx == self.goal_idx:
                return current_node.build_waypoints(), None
            
            d_lookahead = self.config['d_lookahead'] 
            idx = original_idx + int(d_lookahead / self.traj.dstep) # + np.random.uniform(0, 0)
            target, target_idx = self.traj.get_waypoint_bounded(idx)

            mean_action = self.model.controller(current_node, target, self.traj.estim_max_speed[original_idx])
            action, understeer  = self.generate_action(current_node, mean_action) #, original_idx)
            new_node = self.model.generate_child(current_node, action, self.traj)
            self.add_node(new_node)
            current_node = new_node
            print('growing node', current_node, self.traj.estim_max_speed[target_idx], action, target_idx)
            num_growed += 1
            if understeer:
                num_understeer += 1 

        if num_understeer/num_growed > self.config['understeer_thres'] :
            for i in range(num_growed-1):
                if current_node is None:
                    print('Update max speed warning:')
                    break
                idx, dist = current_node.get_idx_dist(self.traj)
                self.traj.estim_max_speed[idx] -= max(1.6, abs(self.traj.estim_max_speed[idx]-current_node.v_x)/8)
                current_node = current_node.parent
        
        return num_understeer, num_growed


    def generate_action(self, node , mean_action): #, target_idx):
        acc, steer = mean_action
# simple clipping right now, adding randomization and gridding later
        max_steer, dynamic_max_steer = self.model.max_steering_angle(node.v_x)

        understeer = False
        if steer > dynamic_max_steer or steer < -dynamic_max_steer:
            understeer = True

        actual_acc = np.clip(acc, self.model.a_min, self.model.a_max)
        actual_steer = np.clip(steer, -max_steer, max_steer)
        # print('target steer, max_steer', steer, max_steer)
        return (actual_acc, actual_steer), understeer
    

    def get_rollback(self, low, high, mu, sigma):
        
        # Convert to standard normal bounds
        a, b = (low - mu) / sigma, (high - mu) / sigma
        generated = int(truncnorm.rvs(a, b, loc=mu, scale=sigma, size=1, random_state=self.rng))
        print(low, high, mu, sigma, generated)
        # Generate a single sample
        return generated

    def get_expansion_node(self):
        choice_nodes = []
        # print('self.node_grid', self.node_grid)
        while len(choice_nodes) == 0:
            rollback = self.get_rollback(0,  self.config['prev_horizon'], self.mean_rollback,  self.config['sigma_expansion_idx'])
            # print('rollback: ', rollback)
            choice_nodes = self.node_grid[rollback]
        speeds = [node.v_x for node in choice_nodes]
        return choice_nodes[np.argmin(speeds)]

        # return np.random.choice(choice_nodes)

    def update_mean_rollback(self, understeer_rate):
        if self.frontier_idx - self.prev_frontier > self.config['d_lookahead']:
            self.mean_rollback =  self.config['mean_rollback'] 
        d_mean_rollback = np.ceil(max(0, (understeer_rate - self.config['understeer_thres'])*40 ))
        self.mean_rollback += d_mean_rollback
        if self.mean_rollback >= self.config['prev_horizon']:
            print('Warning: too many rollback')
            self.mean_rollback = self.config['prev_horizon']
            

        self.prev_frontier = self.frontier_idx
        


    def search(self, start, target_speed, goal_idx = -1):
        if (goal_idx<0) or (goal_idx>=len(self.traj.points)):
            goal_idx = len(self.traj.points) - 1
        self.goal_idx = goal_idx

        self.add_node(start)
        self.visualize_max_speed = target_speed
        self.traj.set_constant_max_speed(target_speed)

        if not start.is_valid(self.traj):
            print("Warning: start point is outside the track.")
            return None
        
        expansion_node = start
        while True:
            ret1, ret2 = self.grow_from_node(expansion_node)
            if isinstance(ret1, list):
                return ret1
            num_understeer, num_growed = ret1, ret2
            self.update_mean_rollback(num_understeer/num_growed)
            expansion_node = self.get_expansion_node()
            print('expansion_node', expansion_node)
            # print('frontier node', self.frontier_node)
        




            
    
    

