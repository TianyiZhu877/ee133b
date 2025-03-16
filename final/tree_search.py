import numpy as np

ESTIM = 0
LEFT = 1
RIGHT = 2


class treeSearch:
    def __init__(self, traj, model, visualizer, config):
        self.model = model
        self.visualizer = visualizer
        self.traj = traj
        self.config = config
    
        self.grid = np.zeros((self.config['prev_horizon'], self.config['longi_resolution']))
        self.node_grid = []
        for i in range(self.config['prev_horizon']):
            new_row = []
            for j in range(self.config['longi_resolution']):
                new_row.append([])
                
            self.node_grid.append(new_row)

        self.frontier_idx = 0
        self.frontier_node = None

        self.visualize_max_speed = 25
        self.visualize = True
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
            # self.frontier_node = node
            new_idx = idx-self.frontier_idx
            if new_idx > self.config['prev_horizon']:
                print('Search tree warning: frontier pushed by two much')
            self.grid = self.grid[:-new_idx, :]
            self.grid = np.vstack((np.zeros((new_idx, self.config['longi_resolution'])), self.grid))

            self.node_grid = self.node_grid[:-new_idx]
            for i in range(new_idx):
                new_row = []
                for j in range(self.config['longi_resolution']):
                    new_row.append([])

                self.node_grid.insert(0, new_row)

            self.frontier_idx = idx
        
        grid_y = (dist+self.traj.max_width)/(self.traj.max_width*2) * self.config['longi_resolution']
        # print(grid_y)
        grid_y = int(np.floor(min(max(0, grid_y), self.config['longi_resolution']-1)))
        grid_x = self.frontier_idx - idx
        # print(grid_x, grid_y)
        self.grid[grid_x, grid_y] += 1
        self.node_grid[grid_x][grid_y].append(node)
        # else:
        #     self.frontier_node = None

    def grow_from_node(self, node, policy = ESTIM):
        current_node = node

        while not self.model.has_no_valid_children(current_node, self.traj):
            original_idx, dist = current_node.get_idx_dist(self.traj)

            if original_idx == self.goal_idx:
                return current_node.build_waypoints()
            
            d_lookahead = self.config['d_lookahead'] 
            idx = original_idx + int(d_lookahead / self.traj.dstep) # + np.random.uniform(0, 0)
            target, target_idx = self.traj.get_waypoint_bounded(idx)

            mean_action = self.model.controller(current_node, target, self.traj.estim_max_speed[original_idx])
            action = self.generate_action(current_node, mean_action, original_idx)
            new_node = self.model.generate_child(current_node, action, self.traj)
            self.add_node(new_node)
            current_node = new_node
            print('growing node', current_node, self.traj.estim_max_speed[target_idx], action, target_idx)


    def generate_action(self, node , mean_action, target_idx):
        acc, steer = mean_action
# simple clipping right now, adding randomization and gridding later
        max_steer, dynamic_max_steer = self.model.max_steering_angle(node.v_x)

        if steer > dynamic_max_steer or steer < -dynamic_max_steer:
            self.traj.estim_max_speed[target_idx] -= max(1, abs(self.traj.estim_max_speed[target_idx]-node.v_x)/8)

        actual_acc = np.clip(acc, self.model.a_min, self.model.a_max)
        actual_steer = np.clip(steer, -max_steer, max_steer)
        # print('target steer, max_steer', steer, max_steer)
        return (actual_acc, actual_steer)
    
    def get_expansion_node(self):
        
        nonzero_indices = np.nonzero(self.grid)
        print(nonzero_indices)
        values = self.grid[nonzero_indices[0], nonzero_indices[1]]
        print(values)

        # Inverse weighting: Higher values should be less likely
        weights = 1 / values
        weights /= weights.sum()  # Normalize to sum to 1

        i = np.random.choice(np.arange(len(weights))) #, p=weights)
        print('nonzero_indices[0][i]][nonzero_indices[1][i]', nonzero_indices[0][i], nonzero_indices[1][i])
        choice_nodes = self.node_grid[nonzero_indices[0][i]][nonzero_indices[1][i]]
        # print(self.node_grid)
        speeds = [node.v_x for node in choice_nodes]
        return choice_nodes[np.argmin(speeds)]

        # return np.random.choice(choice_nodes)



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
            result = self.grow_from_node(expansion_node)
            if result is not None:
                return result
            expansion_node = self.get_expansion_node()
            print('expansion_node', expansion_node)
            # print('frontier node', self.frontier_node)
        




            
    
    

