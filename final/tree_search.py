import numpy as np

class treeSearch:
    def __init__(self, traj, model, visualizer, config):
        self.model = model
        self.visualizer = visualizer
        self.traj = traj
        self.config = config
    
        self.expansion_grid = []
        self.frontier_idx = 0
        self.frontier_node = None
        
    def add_node(self, node, prev_is_frontier = False):
        if node is None:
            self.frontier_node = None
            return 
        
        idx, dist = self.traj.nearest_point(node.position())
        frontier_expaned = False

        if (prev_is_frontier) or (idx > self.frontier_idx):
            self.frontier_node = node
            self.frontier_idx = idx
            frontier_expaned = True
        else:
            self.frontier_node = None


    def generate_action(self, node , action):
        acc, steer = action
# simple clipping right now, adding randomization and gridding later
        max_steer = self.model.max_steering_angle(node.v_x)
        actual_acc = np.clip(acc, self.model.a_min, self.model.a_max)
        actual_steer = np.clip(steer, -max_steer, max_steer)
        # print('target steer, max_steer', steer, max_steer)
        return (actual_acc, actual_steer)

    def search(self, start, target_speed, goal_idx = -1):
        if (goal_idx<0) or (c>=len(self.traj.points)):
            goal_idx = len(self.traj.points) - 1

        self.add_node(start, True)


        if not self.traj.is_inside_track((start.x, start.y)):
            print("Warning: start point is outside the track.")
            return None
        

        while True:
            prev_is_frontier = False

            if self.frontier_node is not None:
                prev_is_frontier = True
                target_idx = self.frontier_idx
                expansion_node = self.frontier_node
            else:
                break
            
            # print(self.frontier_idx)
            # print(expansion_node)
            # print()

            new_node = None
            if not self.model.has_no_valid_children(expansion_node, self.traj):
                mean_action = self.model.controller(expansion_node, self.traj, target_idx, target_speed)
                action = self.generate_action(expansion_node, mean_action)
                new_node = self.model.generate_child(expansion_node, action, self.traj)
            else:
                print("Warning: no valid children found.")
            
            
            self.add_node(new_node, prev_is_frontier)
            # print('frontier node', self.frontier_node)
        
            if new_node is not None:
                self.visualizer.plot_node(new_node, target_speed)




            
    
    

