from trajectory import *
from visualize import Visualizer
from tree_search import treeSearch
from car_model import carModel
from node import Node
import matplotlib.pyplot as plt

# traj = get_sharpe_angle_traj()
def get_starting_point_from_traj(traj, start_speed = 10):
    theta = np.arctan2(traj.tangent[0, 1], traj.tangent[0, 0])
    x = traj.points[0, 0]
    y = traj.points[0, 1]
    return Node(x, y, theta, start_speed,0, 0, 0)

controller_config = {
    'P_acc' : 10
}



tree_search_config = {
    'd_lookahead': 1.2,
    'prev_horizon': 200,
    'understeer_thres': 0.6,
    'sigma_expansion_idx': 10,
    'mean_rollback': 10
    # 'longi_resolution': 30, 
                           }



car_model = carModel(controller_config = controller_config)


# for v in range(1, 35, 5):
#     print(v, car_model.max_steering_angle(v)/3.1416*180)


traj = get_sharpe_angle_traj()
# traj = get_sin_traj()
vis = Visualizer()
vis.plot_trajectory(traj)
search = treeSearch(traj, car_model, vis, tree_search_config)
# print(traj.tangent)
start_node = get_starting_point_from_traj(traj, 5)
result = search.search(start_node, 30)
# vis.show()
if result is not None:
    vis.plot_waypoints(result, 25)
# plt.show()
