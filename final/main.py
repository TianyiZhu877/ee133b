from trajectory import *
from visualize import Visualizer

# traj = get_sharpe_angle_traj()
traj = get_sin_traj()
vis = Visualizer()
vis.plot_trajectory(traj)
vis.show()
