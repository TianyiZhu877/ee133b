#!/usr/bin/env python3
#
#   prmtriangles.py
#
#   Use PRM to find a path around triangular obstacles.
#
#   This is skeleton code. Please fix, especially where marked by "FIXME"!
#
import matplotlib.pyplot as plt
import numpy as np
import random
import time

from math               import pi, sin, cos, sqrt, ceil, dist
from scipy.spatial      import KDTree
from shapely.geometry   import Point, LineString, Polygon, MultiPolygon
from shapely.prepared   import prep

from astar import AStarNode, astar


######################################################################
#
#   Parameters
#
#   FIXME: Define the N/K...
#
# N = 20 #FIXME: Select the number of nodes
# K = 10 # FIXME: Select the number of nearest neighbors

N = 10 #FIXME: Select the number of nodes
K = 5 # FIXME: Select the number of nearest neighbors

random.seed(0)
######################################################################
#
#   World Definitions
#
#   List of obstacles/objects as well as the start/goal.
#
(xmin, xmax) = (0, 12)
(ymin, ymax) = (0, 12)

# Collect all the triangle and prepare (for faster checking).
triangles = prep(MultiPolygon([
    Polygon([[6, 3], [2,  3], [2, 4], [6, 3]]),
    Polygon([[4, 5], [6,  6], [4, 7], [4, 5]]),
    Polygon([[8, 4], [8,  7], [6, 5], [8, 4]]),
    Polygon([[2, 9], [5, 10], [5, 8], [2, 9]])]))

# Define the start/goal states (x, y, theta)
(xstart, ystart) = (5, 1)
(xgoal,  ygoal)  = (4, 11)


######################################################################
#
#   Utilities: Visualization
#
# Visualization Class
class Visualization:
    def __init__(self):
        # Clear the current, or create a new figure.
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.grid(True)
        plt.gca().axis('on')
        plt.gca().set_xlim(xmin, xmax)
        plt.gca().set_ylim(ymin, ymax)
        plt.gca().set_aspect('equal')

        # Show the triangles.
        for poly in triangles.context.geoms:
            plt.plot(*poly.exterior.xy, 'k-', linewidth=2)

        # Show.
        self.show()

    def show(self, text = ''):
        # Show the plot.
        plt.pause(0.001)
        # If text is specified, print and wait for confirmation.
        if len(text)>0:
            input(text + ' (hit return to continue)')

    def drawNode(self, node, *args, **kwargs):
        plt.plot(node.x, node.y, *args, **kwargs)

    def drawEdge(self, head, tail, *args, **kwargs):
        plt.plot((head.x, tail.x),
                 (head.y, tail.y), *args, **kwargs)

    def drawPath(self, path, *args, **kwargs):
        for i in range(len(path)-1):
            self.drawEdge(path[i], path[i+1], *args, **kwargs)


######################################################################
#
#   Node Definition
#
class Node(AStarNode):
    def __init__(self, x, y):
        # Setup the basic A* node.
        super().__init__()

        # Define/remember the state/coordinates (x,y).
        self.x = x
        self.y = y

    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return ("<Point %5.2f,%5.2f>" % (self.x, self.y))

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    def intermediate(self, other, alpha):
        return Node(self.x + alpha * (other.x - self.x),
                    self.y + alpha * (other.y - self.y))

    # Return a tuple of coordinates, used to compute Euclidean distance.
    def coordinates(self):
        return (self.x, self.y)

    # Compute the relative Euclidean distance to another node.
    def distance(self, other):
        # FIXME: compute and return the distance.
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    ###############
    # A* functions:
    # Actual and Estimated costs.
    def costToConnect(self, other):
        return self.distance(other)

    def costToGoEst(self, other):
        return self.distance(other)

    ################
    # PRM functions:
    # Check whether in free space.
    def inFreespace(self):
        return triangles.disjoint(Point(self.coordinates()))

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other):
        line = LineString([self.coordinates(), other.coordinates()])
        return triangles.disjoint(line)


######################################################################
#
#   PRM Functions
#
# Create the list of nodes.
def createNodes(N):
    # FIXME: create the list of valid nodes sampling uniformly in x and y.
    # Add nodes sampled uniformly across the space.
    nodes = []
    sampled = 0
    while len(nodes) < N:
        sampled += 1
        if (sampled > N*10):
            print(f"Warning: to many points sampled lies on obstacles, ditched with {len(nodes)} nodes")
            return nodes
        
        new_node = Node(random.uniform(xmin, xmax), random.uniform(ymin, ymax))
        if new_node.inFreespace():
            nodes.append(new_node)
    
    return nodes
            

# Connect the nearest neighbors
def connectNearestNeighbors(nodes, K):
    # Clear any existing neighbors.  Use a set to add below.
    for node in nodes:
        node.neighbors = set()

    # Determine the indices for the K nearest neighbors.  Distance is
    # computed as the Euclidean distance of the coordinates.  This
    # also reports the node itself as the closest neighbor, so add one
    # extra here and ignore the first element below.
    X = np.array([node.coordinates() for node in nodes])
    [dist, idx] = KDTree(X).query(X, k=(K+1))

    # Add the edges.  Ignore the first neighbor (being itself).
    for i, nbrs in enumerate(idx):
        for n in nbrs[1:]:
            if nodes[i].connectsTo(nodes[n]):
                nodes[i].neighbors.add(nodes[n])
                nodes[n].neighbors.add(nodes[i])

# Post Process the Path
def PostProcess(path):
    # FIXME: Remove nodes in the path than can be skipped without collisions
    n = len(path)
    if n == 0:
        return []
    # if n == 1:
    
    new_path = [path[0]]

    current_ptr = 0
    while current_ptr<(n-1):
        current = path[current_ptr]
        next_ptr = current_ptr+1
        while (next_ptr < n-1) and (current.connectsTo(path[next_ptr+1])):
            next_ptr += 1
        new_path.append(path[next_ptr])
        current_ptr = next_ptr
    
    return new_path
        
        



######################################################################
#
#  Main Code
#
def main():
    # Report the parameters.
    print('Running with', N, 'nodes and', K, 'neighbors.')

    # Create the figure.
    visual = Visualization()

    # Create the start/goal nodes.
    startnode = Node(xstart, ystart)
    goalnode  = Node(xgoal,  ygoal)

    # Show the start/goal nodes.
    visual.drawNode(startnode, color='orange', marker='o')
    visual.drawNode(goalnode,  color='purple', marker='o')
    visual.show("Showing basic world")


    # Create the list of nodes.
    print("Sampling the nodes...")
    tic = time.time()
    nodes = createNodes(N)
    toc = time.time()
    print("Sampled the nodes in %fsec." % (toc-tic))

    # Show the sample nodes.
    for node in nodes:
        visual.drawNode(node, color='k', marker='x')
    visual.show("Showing the nodes")

    # Add the start/goal nodes.
    nodes.append(startnode)
    nodes.append(goalnode)


    # Connect to the nearest neighbors.
    print("Connecting the nodes...")
    tic = time.time()
    connectNearestNeighbors(nodes, K)
    toc = time.time()
    print("Connected the nodes in %fsec." % (toc-tic))

    # Show the neighbor connections.
    for (i,node) in enumerate(nodes):
        for neighbor in node.neighbors:
            if neighbor not in nodes[:i]:
                visual.drawEdge(node, neighbor, color='g', linewidth=0.5)
    visual.show("Showing the full graph")


    # Run the A* planner.
    print("Running A*...")
    tic = time.time()
    path = astar(nodes, startnode, goalnode)
    toc = time.time()
    print("Ran A* in %fsec." % (toc-tic))

    # If unable to connect, show the part explored.
    if not path:
        print("UNABLE TO FIND A PATH")
        for node in nodes:
            if node.done:
                visual.drawNode(node, color='r', marker='o')
        visual.show("Showing DONE nodes")
        return

    # Show the path.
    visual.drawPath(path, color='r', linewidth=2)
    visual.show("Showing the raw path")


    # Post Process the path.
    path = PostProcess(path)

    # Show the post-processed path.
    visual.drawPath(path, color='b', linewidth=2)
    visual.show("Showing the post-processed path")


if __name__== "__main__":
    main()
