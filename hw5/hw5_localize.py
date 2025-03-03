#!/usr/bin/env python3
#
#   hw5_localize.py
#
#   Homework 5 code framework to localize a robot in a grid...
#
#   Places to edit are marked as FIXME.
#
import numpy as np

from hw5_utilities import Visualization, Robot


#
#  Define the Walls
#
w = ['xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx',
     'x               xx             xx               x',
     'x                xx           xx                x',
     'x                 xx         xx                 x',
     'x        xxxx      xx       xx                  x',
     'x        x  xx      xx     xx                   x',
     'x        x   xx      xx   xx      xxxxx         x',
     'x        x    xx      xx xx     xxx   xxx       x',
     'x        x     xx      xxx     xx       xx      x',
     'x        x      xx      x      x         x      x',
     'x        x       xx           xx         xx     x',
     'x        x        x           x           x     x',
     'x        x        x           x           x     x',
     'x        x        x           x           x     x',
     'x                 xx         xx           x     x',
     'x                  x         x                  x',
     'x                  xx       xx                  x',
     'x                   xxx   xxx                   x',
     'x                     xxxxx         x           x',
     'x                                   x          xx',
     'x                                   x         xxx',
     'x            x                      x        xxxx',
     'x           xxx                     x       xxxxx',
     'x          xxxxx                    x      xxxxxx',
     'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx']

walls = np.array([[1.0*(c == 'x') for c in s] for s in w])
rows  = np.size(walls, axis=0)
cols  = np.size(walls, axis=1)

def withinBound(row, col):
    return row<rows and row>=0 and col<cols and col>=0

def validPosition(row, col):
    return withinBound(row, col) and walls[row][col]<0.5


#
# getNonzeroTransitions
#
# return:
# [ (p((new_row,new_col) | (row, col), (drow, dcol)) , new_rol, new_col) ]
# for all new_rol, new_col such that p != 0
# 

def getNonzeroTransitions(row, col, drow, dcol, pCmdUsed):
    new_row, new_col = row+drow, col+dcol
    if not validPosition(new_row, new_col):
        return [(1, row, col)]
    return [(1-pCmdUsed, row, col), (pCmdUsed, new_row, new_col)]

#
#  Prediction
#
#    bel         Grid of probabilities (current belief)
#    drow, dcol  Delta in row/col
#    pCmdUsed    Modeled probability of command executing
#    prd         Grid of probabilities (prediction)
#

def computePrediction(bel, drow, dcol, pCmdUsed = 1, expect_jump = 0):
    # Prepare an empty prediction grid.
    prd = np.zeros((rows,cols))

    # Determine the new probablities (remember to consider walls).
    # FIXME...
    for row_tn1 in range(rows):
        for col_tn1 in range(cols):
            empty_cells = np.sum(1.0 - walls)
            # print(empty_cells)
            prd += (bel[row_tn1, col_tn1] * (1-walls) * expect_jump / empty_cells)
            # dprd = bel[row_tn1, col_tn1] * np.sum((1-walls) * expect_jump / empty_cells)
            # print(walls)
            for (p, row_t, col_t) in getNonzeroTransitions(row_tn1, col_tn1, drow, dcol, pCmdUsed):
                if withinBound(row_t, col_t):
                    prd[row_t, col_t] += (bel[row_tn1, col_tn1] * p * (1-expect_jump))
                    # dprd +=  (bel[row_tn1, col_tn1] * p * (1-expect_jump))
                    # print(p * (1-expect_jump))
                else:
                    print("out of bound!!!!")
            # print("total dprd:", dprd, bel[row_tn1, col_tn1])
            

    # Return the prediction grid
    return prd


#
#  Measurement Update (Correction)
#
#    prior       Grid of prior probabilities (belief)
#    probSensor  Grid of probabilities that (sensor==True)
#    sensor      Value of sensor
#    post        Grid of posterior probabilities (updated belief)
#
def updateBelief(prior, probSensor, sensor):
    # Create the posterior belief.
    # post = FIXME...
    post = probSensor * prior if sensor else (1-probSensor) * prior

    # Normalize.
    s = np.sum(post)
    if (s == 0.0):
        print("LOST ALL BELIEF - THIS SHOULD NOT HAPPEN!!!!")
    else:
        post = (1.0/s) * post
    return post


def shiftGrid(arr, drow, dcol):
    shifted = np.ones_like(arr)  
    rows, cols = arr.shape
    
    row_start = max(0, drow)
    row_end = min(rows, rows + drow)
    col_start = max(0, dcol)
    col_end = min(cols, cols + dcol)
    
    arr_row_start = max(0, -drow)
    arr_row_end = min(rows, rows - drow)
    arr_col_start = max(0, -dcol)
    arr_col_end = min(cols, cols - dcol)
    
    shifted[row_start:row_end, col_start:col_end] = arr[arr_row_start:arr_row_end, arr_col_start:arr_col_end]
    
    return shifted

def getDir(d):
    if d==0:
        return 0
    return 1 if d>0 else -1

#
#  Pre-compute the Sensor Probability Grid
#
#    drow, dcol   Direction in row/col
#    pSenDist     List of probabilities that sensor triggers at dist=(index+1)
#    prob         Grid of probabilities that (sensor==True)
#
def precomputeSensorProbability(drow, dcol, pSenDist = [1.0]):
    # Prepare an empty probability grid.
    prob = np.zeros((rows, cols)) 


    # Pre-compute the sensor probability on the grid.
    # FIXME...
    drow, dcol = getDir(drow), getDir(dcol)
    
    # for row in rows:
    #     for col in cols:
    for i, detect_p in enumerate(pSenDist):
        prob = np.maximum(prob, shiftGrid(walls, -(i+1)*drow, -(i+1)*dcol) * detect_p)

    # Return the computed grid.
    return prob


# 
#
#  Main Code
#
def main():
    # FIXME... PICK WHAT THE "REALITY" SHOULD SIMULATE:
    # Initialize the robot simulation.
    # 1a 
    # robot=Robot(walls)
    # robot=Robot(walls, row=12, col=26)  # 1b 
    # robot=Robot(walls, row=12, col=26, pSensor=[0.9,0.6,0.3])       # 2  
    # robot=Robot(walls, row=15, col=47, pSensor=[0.9,0.6,0.3], pCommand=0.8)     # 3  
    robot=Robot(walls, row= 7, col=12, pSensor=[0.9,0.6,0.3], pCommand=0.8, kidnap=True)        # 4  
    # Or to play:
    #    robot=Robot(walls, pSensor=[0.9,0.6,0.3], pCommand=0.8)


    # Initialize your localization parameters.
    pSenDist = [0.9,0.6,0.3] # FIXME... PICK WHAT YOUR LOCALIZATION SHOULD ASSUME
    pCmdUsed = 0.8 # FIXME... PICK WHAT YOUR LOCALIZATION SHOULD ASSUME
    expect_jump = 1e-2

    # Report.
    print("Localization is assuming pSenDist = " + str(pSenDist) +
          ", pCmdUsed = " + str(pCmdUsed))


    # Initialize the figure.
    visual = Visualization(walls, robot)


    # Pre-compute the probability grids for each sensor reading.
    probUp    = precomputeSensorProbability(-1,  0, pSenDist)
    probRight = precomputeSensorProbability( 0,  1, pSenDist)
    probDown  = precomputeSensorProbability( 1,  0, pSenDist)
    probLeft  = precomputeSensorProbability( 0, -1, pSenDist)

    # Show the sensor probability maps.
    # visual.Show(probUp)
    # input("Probability of proximal sensor up reporting True")
    # visual.Show(probRight)
    # input("Probability of proximal sensor right reporting True")
    # visual.Show(probDown)
    # input("Probability of proximal sensor down reporting True")
    # visual.Show(probLeft)
    # input("Probability of proximal sensor left reporting True")


    # Start with a uniform belief grid.
    bel = (1.0 - walls) / np.sum(1.0 - walls)

    # Loop continually.
    while True:
        # Show the current belief.  Also show the actual position.
        visual.Show(bel, markRobot=True)

        # Get the command key to determine the direction.
        while True:
            key = input("Cmd (q=quit, w=up, s=down, a=left, d=right) ?")
            if   (key == 'q'):  return
            elif (key == 'w'):  (drow, dcol) = (-1,  0) ; break
            elif (key == 's'):  (drow, dcol) = ( 1,  0) ; break
            elif (key == 'a'):  (drow, dcol) = ( 0, -1) ; break
            elif (key == 'd'):  (drow, dcol) = ( 0,  1) ; break

        # Move the robot in the simulation.
        robot.Command(drow, dcol)


        # Compute a prediction.
        prd = computePrediction(bel, drow, dcol, pCmdUsed, expect_jump=expect_jump)
        #visual.Show(prd)
        #input("Showing the prediction")

        # Check the prediction.
        if abs(np.sum(prd) - 1.0) > 1e-12:
            print("WARNING: Prediction does not add up to 100%", np.sum(prd))


        # Correct the prediction/execute the measurement update.
        bel = prd
        bel = updateBelief(bel, probUp,    robot.Sensor(-1,  0))
        bel = updateBelief(bel, probRight, robot.Sensor( 0,  1))
        bel = updateBelief(bel, probDown,  robot.Sensor( 1,  0))
        bel = updateBelief(bel, probLeft,  robot.Sensor( 0, -1))


if __name__== "__main__":
    main()
