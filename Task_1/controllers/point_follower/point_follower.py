#!/usr/bin/env python3

#Webots import statments
from controller import Robot
from controller import GPS , Motor ,  InertialUnit
import math

## ------------------------------------------------------------------------
#Edit point_follower and Sketch functions for Task 1, 2 , 3

def point_follower(current,goal):
    # Implement a Controller to reach the goal.

    # current is [x,y,theta]
    # i.e x = current[0], y = current[1], theta = current[2]  
    # goal locations can be fetched the same way
    # goal is [x,y,theta]
      
    # leftSpeed is a float
    # rightSpeed is a float

    # Controller code here to reach from "current" location to "goal" location. 
    
    # Testing
    #print("diff x 0: ", abs(current[0] - (goal[0])[0]))
    #print("diff y 0: ", abs(current[1] - (goal[0])[1]))
    #print("diff theta 0: ", abs(current[2] - (goal[0])[2]))
    
    #print("diff x 1: ", abs(current[0] - (goal[1])[0]))
    #print("diff y 1: ", abs(current[1] - (goal[1])[1]))
    #print("diff theta 1: ", abs(current[2] - (goal[1])[2]))
    
    #print("diff x 2: ", abs(current[0] - (goal[2])[0]))
    #print("diff y 2: ", abs(current[1] - (goal[2])[1]))
    #print("diff theta 2: ", abs(current[2] - (goal[2])[2]))
    
    #print("diff x 3: ", abs(current[0] - (goal[3])[0]))
    #print("diff y 3: ", abs(current[1] - (goal[3])[1]))
    #print("diff theta 3: ", abs(current[2] - (goal[3])[2]))
    if  abs(current[1] - (goal[0])[1]) < 0.1 and abs(current[0] - (goal[0])[0]) < 0.1 and abs(current[2] - (goal[0])[2]) > 0.05:
        print("up")
        leftSpeed = -5.0 
        rightSpeed = 5.0
    elif abs(current[0] - (goal[1])[0]) < 0.1 and abs(current[1] - (goal[1])[1]) < 0.1 and abs(current[2] - (goal[1])[2]) > 0.05:
        print("left")
        leftSpeed = -5.0 
        rightSpeed = 5.0
    elif abs(current[1] - (goal[2])[1]) < 0.2 and abs(current[0] - (goal[2])[0]) < 0.1 and abs(current[2] - (goal[2])[2]) > 0.05:
        print("down")
        leftSpeed = -5.0 
        rightSpeed = 5.0
    elif abs(current[0] - (goal[3])[0]) < 0.2 and abs(current[1] - (goal[3])[1]) < 0.2 and abs(current[2] - (goal[3])[2]) > 0.05:
        print("right")
        leftSpeed = 0
        rightSpeed = 0  
    else:
        leftSpeed = 4.8 
        rightSpeed = 4.8
    
    # Sample velocities of 1.0 provided to make robot move straight by default. 
    return leftSpeed,rightSpeed

def Sketch():
    # Use this function to calculate waypoints that can trace the given curve in the world. 
    # This is optional you can also impelement everything in point_follower function.
    # Clue: You can think of equations for the curve and create waypoints to follow it.
    # How to add additional goal points to the goal vector :
    # Eg: to add [1,1,0] as goal
    # goal =[]
    # goal.append([1,1,0])
    # then to add [1,2,1.57] as goal
    # goal.append([1,2,1.57])
    # To access 1st goal : goal[0], 2nd goal : goal[1] and so on..
    goal = [] # [x,y,theta]
    goal.append([4, -4, 1.57])
    goal.append([4, 4, 3.14])
    goal.append([-4, 4, -1.57])
    goal.append([-4, -4, 0])
    return goal

## ------------------------------------------------------------------------

#Initializing robot to access sensor data from the robot.
robot = Robot()
timestep = int(robot.getBasicTimeStep()) # Defined timestep (in msec) to enable sensors and define sensor frequency.

# Initialize and Enable GPS object to get X,Y location of robot
gps = robot.getGPS("gps")
gps.enable(timestep) # x, y, z location received at time difference equat to timestep.

# Initialize and Enable IMU object to get theta (orientation) of robot
imu = robot.getInertialUnit("imu")
imu.enable(timestep) # Theta recieved at time difference equat to timestep.


# Initialize and Enable robot wheels
wheels_names = ["wheel1", "wheel2", "wheel3", "wheel4"]
wheels = []
for i in range(4):
    wheels.append(robot.getMotor(wheels_names[i]))
    wheels[-1].setPosition(float('inf')) # setting max position limit of each wheel to infinity to convert it to velocity mode 
    wheels[-1].setVelocity(0.0) # Setting zero velocity for all the wheels.

if __name__=="__main__":

    #Optional Edit here ------------------------------------------------------------
    #Call the Sketch function here if you want to generate vector of goals just once
    # ------------------------------------------------------------------------------
    
    while robot.step(timestep) != -1:

        # Fetch current position of robot using GPS and IMU : x,y,theta
        current = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]]    
        print("current x, y, theta of robot: ",current)
        
        #comment this default goal location if you caculate your own set of goals vector 
        goal = Sketch() # initial goal to initialize goal array

        ## ------------------------------------------------------------------------------
        #Edit here 
        # Call the Sketch function here if you want to generate vector of goals continuously
        # Use point_follower controller to trace the curve using the above generated waypoints   
        # point_follower should return leftSpeed and rightSpeed 
        leftSpeed,rightSpeed = point_follower(current,goal)
        ## ------------------------------------------------------------------------------

        # Setting velocities to each wheel based on calculation.
        wheels[0].setVelocity(leftSpeed) # Front left wheel
        wheels[1].setVelocity(rightSpeed) # Front right wheel
        wheels[2].setVelocity(leftSpeed) # Rear left wheel
        wheels[3].setVelocity(rightSpeed) # Rear right wheel
         
