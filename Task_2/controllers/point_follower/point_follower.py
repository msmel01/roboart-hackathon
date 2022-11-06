#!/usr/bin/env python3

#Webots import statments
from controller import Robot
from controller import GPS , Motor , InertialUnit
import math
import numpy as np
import matplotlib.pyplot as plt
import sympy as sym

## ------------------------------------------------------------------------
#Edit point_follower and Sketch functions for Task 1, 2 , 3

INC = 0.1

def sin_trajectory_func(x):
    # Model is Asin(Bx)
    amp = 9.9 / 2
    phase = 2 * np.pi / 9.9
    return amp * np.sin(phase * x)
   
    
def sin_derivative(x):
    phase = 2 * np.pi / 9.9
    return np.pi * np.cos(phase * x)


def calculate_angle_at(x, y): 
    return np.arctan(sin_derivative(x))
    
    
def point_follower(current,goal):
    # Error threshold
    error_threshold = 0.01
    
    current_angle = current[2]
    goal_angle = goal[2]
    error = np.abs(goal_angle - current_angle)
    # error = math.atan2((current[1]-goal[1]),(current[0]-goal[0]))
    
    if goal_angle < 0:
        is_neg_angle = True
    else:
        is_neg_angle = False
        
    # is_neg_angle = True if goal_angle < 0 else False
    
    # print(current_angle)
    # print(error)
    # print('--------------')
    
    # Counter-clockwise = left negative, right positive
    # Clockwise = left positive, right negative
    if np.abs(error) > error_threshold:
        if is_neg_angle:
            return -0.1, 0.1, False, False
        else:
            return 0.1, -0.1, False, False
    
    error_dist = np.sum(np.square(np.subtract(current[:2], goal[:2])))
    
    is_neg_dist = True if current[0] > goal[0] else False
    
    if error_dist > 0.01:
        if is_neg_dist:
            
            return INC/2, -INC/2, False, True
        else:
            return -INC/2, -INC/2, False, False
        
    left_speed = -INC/4
    right_speed = -INC/4
    # Sample velocities of 1.0 provided to make robot move straight by default. 
    return left_speed, right_speed, True, False


def Sketch():
    '''
    Dimensions of the plane are 9.9 by 9.9.
    where (0,0) is the center of the plane and 0 degrees is facing East.
    The trajectory of the robot is a sinosidal curve, where:
        Min value is ~ -2.475 (to the left), ~ -4.95 (at the bottom)
        Max value is ~ 2.475 (to the right), ~ 4.95 (at the top)
    Model A*sin(Bx + C) + D
    The amplitude A of the sin function is 4.95.
    There is no vertical shift so D = 0.
    There is no horizontal shift so C = 0.
    The period of the function is 9.9 = 2PI/B -> B = 2PI/9.9 = 0.634
    Therefore the model is 4.95sin(0.634x)
    '''

    delta = 0.05 # decrease does better
    x = np.arange(-9.9/2+delta, 9.9/2, delta)
    y = [sin_trajectory_func(pnt) for pnt in x]    
    angle = [calculate_angle_at(x,y) for x, y in zip(x, y)]
    # exp = sym.expand(sin_trajectory_func)
    # z = sym.symbols('z')
    # angle = sym.diff(9.9 / 2 * sin(2 * 3.14 / 9.9 * z), z)
    # print(sym.diff(sym(9.9) / sym(2) * sym.sin(sym(2) * sym('pi') / sym(9.9) * z), z))
    # plt.plot(x, angle)
    # plt.xlim(-9.9/2, -9.9/2+1)
    # plt.show()
    waypoints = list(zip(x, y, angle))
    
    return waypoints

## ------------------------------------------------------------------------

#Initializing robot to access sensor data from the robot.
robot = Robot()
timestep = int(robot.getBasicTimeStep()) # Defined timestep (in msec) to enable sensors and define sensor frequency.

# Initialize and Enable GPS object to get X,Y location of robot
gps = robot.getDevice("gps")
gps.enable(timestep) # x, y, z location received at time difference equat to timestep.

# Initialize and Enable IMU object to get theta (orientation) of robot
imu = robot.getDevice("imu")
imu.enable(timestep) # Theta recieved at time difference equat to timestep.


# Initialize and Enable robot wheels
wheels_names = ["wheel1", "wheel2", "wheel3", "wheel4"]
wheels = []
for i in range(4):
    wheels.append(robot.getDevice(wheels_names[i]))
    wheels[-1].setPosition(float('inf')) # setting max position limit of each wheel to infinity to convert it to velocity mode 
    wheels[-1].setVelocity(0.0) # Setting zero velocity for all the wheels.

if __name__=="__main__":

    #Optional Edit here ------------------------------------------------------------
    #Call the Sketch function here if you want to generate vector of goals just once
    # ------------------------------------------------------------------------------
    waypoints = Sketch()
    idx = 0
    
    while robot.step(timestep) != -1 and idx < len(waypoints):
        
        goal = waypoints[idx]
        # Fetch current position of robot using GPS and IMU : x,y,theta
        current = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]]    
        # print("current x, y, theta of robot: ",current)
        
        #comment this default goal location if you caculate your own set of goals vector 
        # goal = waypoints[0] # initial goal to initialize goal array

        ## ------------------------------------------------------------------------------
        #Edit here 
        # Call the Sketch function here if you want to generate vector of goals continuously
        # Use point_follower controller to trace the curve using the above generated waypoints   
        # point_follower should return leftSpeed and rightSpeed 
        leftSpeed, rightSpeed, isAligned, isStuck = point_follower(current, goal)
        ## ------------------------------------------------------------------------------
        
        if isAligned:
            print('Aligned')
            idx += 1
            
            print('Next target point is {}'.format(goal))
            
        if isStuck:
            print('Im stuck')
            idx += 1
            
            print('Next target point is {}'.format(goal))

        
        # Align angles first
        
        # Align x, y coordinates 
        # leftSpeed = 1
        # rightSpeed = -1
        
        # Setting velocities to each wheel based on calculation.
        wheels[0].setVelocity(leftSpeed) # Front left wheel
        wheels[1].setVelocity(rightSpeed) # Front right wheel
        wheels[2].setVelocity(leftSpeed) # Rear left wheel
        wheels[3].setVelocity(rightSpeed) # Rear right wheel
         
