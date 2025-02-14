#!/usr/bin/env python3
#
#   esttriangles.py
#
#   Use EST to find a path around triangular obstacles.
#
#   This is skeleton code. Please fix, especially where marked by "FIXME"!
#
import matplotlib.pyplot as plt
import numpy as np
import random
import time

from math               import pi, sin, cos, atan2, sqrt, ceil, dist
from scipy.spatial      import KDTree
from shapely.geometry   import Point, LineString, Polygon, MultiPolygon
from shapely.prepared   import prep


######################################################################
#
#   Parameters
#
#   Define the step size.  Also set the maximum number of nodes.
#
DSTEP = 0.5     # FIXME

# Maximum number of nodes.
NMAX = 1500

# heading_random_generator = lambda placeholder: np.random.uniform(-pi, pi)
heading_random_generator = lambda mean: np.random.normal(mean, pi/2)
# heading_random_generator = lambda mean: np.random.normal(mean, pi/8)

scale = 1
np.random.seed(10)
######################################################################
#
#   World Definitions
#
#   List of obstacles/objects as well as the start/goal.
#
(xmin, xmax) = (0, 10)
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
class Node:
    def __init__(self, x, y):
        # Define a parent (cleared for now).
        self.parent = None

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
        return dist(self.coordinates(), other.coordinates())

    ################
    # Collision functions:
    # Check whether in free space.
    def inFreespace(self):
        if (self.x <= xmin or self.x >= xmax or
            self.y <= ymin or self.y >= ymax):
            return False
        return triangles.disjoint(Point(self.coordinates()))

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other):
        line = LineString([self.coordinates(), other.coordinates()])
        return triangles.disjoint(line)


######################################################################
#
#   EST Functions
#
def est(startnode, goalnode, visual):
    # Start the tree with the startnode (set no parent just in case).
    startnode.parent = None
    tree = [startnode]

    # Function to attach a new node to an existing node: attach the
    # parent, add to the tree, and show in the figure.
    def addtotree(oldnode, newnode):
        newnode.parent = oldnode
        tree.append(newnode)
        visual.drawEdge(oldnode, newnode, color='g', linewidth=1)
        visual.show()

    # Loop - keep growing the tree.
    while True:
        # Determine the local density by the number of nodes nearby.
        # KDTree uses the coordinates to compute the Euclidean distance.
        # It returns a NumPy array, same length as nodes in the tree.
        X = np.array([node.coordinates() for node in tree])
        kdtree  = KDTree(X)
        numnear = kdtree.query_ball_point(X, r=1.5*DSTEP, return_length=True)

        # Directly determine the distances to the goal node.
        distances = np.array([node.distance(goalnode) for node in tree])

        # Select the node from which to grow, which minimizes some metric.
        # FIXME:
        min_indices = np.argwhere(np.min(numnear+scale*distances) == numnear+scale*distances).flatten()
        # print(numnear, min_indices)
        grownode = tree[np.random.choice(min_indices)]


        # Check the incoming heading, potentially to bias the next node.
        if grownode.parent is None:
            heading = 0
        else:
            heading = atan2(grownode.y - grownode.parent.y,
                            grownode.x - grownode.parent.x)

        nextnode = None
        # Find something nearby: keep looping until the tree grows.
        while nextnode is None:
            # Pick the next node randomly.
            # FIXME:
            theta = heading_random_generator(heading)
            candidate = Node(grownode.x + DSTEP*cos(theta), grownode.y + DSTEP*sin(theta))

            # Try to connect.
            if candidate.inFreespace() and grownode.connectsTo(candidate):
                nextnode = candidate

        addtotree(grownode, nextnode)   

        # Once grown, also check whether to connect to goal.
        # FIXME...
        if (nextnode.distance(goalnode) <= DSTEP) and (nextnode.connectsTo(goalnode)):
            addtotree(nextnode, goalnode)
            break

        # Check whether we should abort - too many nodes.
        if (len(tree) >= NMAX):
            print("Aborted with the tree having %d nodes" % len(tree))
            return None

    # Build the path.
    path = [goalnode]
    while path[0].parent is not None:
        path.insert(0, path[0].parent)

    # Report and return.
    print("Finished  with the tree having %d nodes" % len(tree))
    return path


# Post process the path.
def PostProcess(path):
    i = 0
    while (i < len(path)-2):
        if path[i].connectsTo(path[i+2]):
            path.pop(i+1)
        else:
            i = i+1


######################################################################
#
#  Main Code
#
def main():
    # Report the parameters.
    print('Running with step size ', DSTEP, ' and up to ', NMAX, ' nodes.')

    # Create the figure.
    visual = Visualization()

    # Create the start/goal nodes.
    startnode = Node(xstart, ystart)
    goalnode  = Node(xgoal,  ygoal)

    # Show the start/goal nodes.
    visual.drawNode(startnode, color='orange', marker='o')
    visual.drawNode(goalnode,  color='purple', marker='o')
    visual.show("Showing basic world")


    # Run the EST planner.
    print("Running EST...")
    path = est(startnode, goalnode, visual)

    # If unable to connect, just note before closing.
    if not path:
        visual.show("UNABLE TO FIND A PATH")
        return

    # Show the path.
    visual.drawPath(path, color='r', linewidth=2)
    visual.show("Showing the raw path")


    # Post process the path.
    PostProcess(path)

    # Show the post-processed path.
    visual.drawPath(path, color='b', linewidth=2)
    visual.show("Showing the post-processed path")


if __name__== "__main__":
    main()
