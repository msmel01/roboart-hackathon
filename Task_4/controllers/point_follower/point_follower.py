#!/usr/bin/env python3

#Webots import statments
from controller import Robot
from controller import Keyboard , GPS , Motor ,  InertialUnit
import math

## ------------------------------------------------------------------------
# Edit keyboard_follower function to control robot using your choice of keyboard keys 

def keyboard_follower(key):
    # Assign different direction of motions with different key values.

    # key is an integer value
    # leftSpeed is a float
    # rightSpeed is a float


    leftSpeed = 1.0
    rightSpeed = 1.0
    # Sample velocities provided to make robot move straight. 
    return leftSpeed,rightSpeed

# Edit point_follower and Sketch functions similar to Task 1/2/3

def point_follower(current,goal):
    # Implement a Controller to reach the goal.

    # current is [x,y,theta]
    # i.e x = current[0], y = current[1], theta = current[2]  
    # goal locations can be fetched the same way
    # goal is [x,y,theta]
      
    # leftSpeed is a float
    # rightSpeed is a float

    # Controller code here to reach from "current" location to "goal" location. 

    leftSpeed = 1.0 
    rightSpeed = 1.0
    # Sample velocities of 1.0 provided to make robot move straight by default. 
    return leftSpeed,rightSpeed

def Sketch():
    # Use this function to calculate waypoints that can trace any curve of your choice  
    # This is optional you can also impelement everything in point_follower function.
    # Clue: You can think of equations for the curve and create waypoints to follow it.
    # How to add additional goal points to the goal vector :
    # Eg: to add [1,1,0] as goal
    # goal =[]
    # goal.append([1,1,0])
    # then to add [1,2,1.57] as goal
    # goal.append([1,2,1.57])
    # To access 1st goal : goal[0], 2nd goal : goal[1] and so on..
    goal = [0,0,0] # [x,y,theta]
    return goal

## ------------------------------------------------------------------------

#Initializing robot to access sensor data from the robot.
robot = Robot()
timestep = int(robot.getBasicTimeStep()) # Defined timestep (in msec) to enable sensors and define sensor frequency.

# Initialize and Enable Keyboard object to get X,Y location of robot
keyboard=Keyboard()
keyboard.enable(timestep)

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

    #Optional Edit here if using Option 2 ------------------------------------------
    #Call the Sketch function here if you want to generate vector of goals just once
    # ------------------------------------------------------------------------------

    while robot.step(timestep) != -1:

        # x,y,theta
        current = [gps.getValues()[0],gps.getValues()[1],imu.getRollPitchYaw()[2]]    
        print("current x, y, theta of robot: ",current)
        goal = [0,0,0] # initial goal to initialize goal array

        ## ------------------------------------------------------------------------
        #Edit here to control using Keyboard
        # Use keyboard_follower function to assign different direction of motions with different key values.
        # keyboard_follower should return leftSpeed and rightSpeed 
        key=keyboard.getKey() # getting currently pressed key on the keyboard.
        #print("Key pressed: ", key)
        leftSpeed,rightSpeed = keyboard_follower(key)
        ## ------------------------------------------------------------------------

        # Setting velocities to each wheel based on calculation.
        wheels[0].setVelocity(leftSpeed) # Front left wheel
        wheels[1].setVelocity(rightSpeed) # Front right wheel
        wheels[2].setVelocity(leftSpeed) # Rear left wheel
        wheels[3].setVelocity(rightSpeed) # Rear right wheel
         
