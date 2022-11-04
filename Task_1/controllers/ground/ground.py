
#Webots import statments
from controller import Display
from controller import Supervisor
from controller import Keyboard

TIME_STEP = 64

# size of full ground in meters.
GROUND_X = 9.9
GROUND_Y = 9.9

# Supervisor node used to fetch location of moving robot. 
super = Supervisor() 

# Enable Keyboard to allow robot to draw / undraw based on robot keys 
keyboard=Keyboard()
keyboard.enable(TIME_STEP) # Keyboard will provide feedback with time difference equal to timestep.

# Handler to canvas backgroud device 
display = super.getDevice("ground_display")

# Fetch properties of above Display
width = display.getWidth()
height = display.getHeight()

# Get access to robot handle to access its tree.
mybot = super.getFromDef("diff_robot")

# Get translation field to now robot's current location. 
translationField = mybot.getField("translation")

# Load Curve background from an image. 
background = display.imageLoad("../../worlds/textures/square.png")
display.imagePaste( background, 0, 0) # Adding background to the display node to make changes on the background.

# // set the pen to remove the texture
display.setAlpha( 0.0)

draw = True # Flag to enable and disable drawing.

while super.step(TIME_STEP) != -1:

  # get currently pressed key on the keyboard.    
  key=keyboard.getKey()
  #print("Key pressed: ", key)
  if (key==80): # P press returns 80 on key variable 
    print ('P is pressed')
    draw=True #If 'P' press enable drawing
  elif(key==79): # O press returns 79 on key variable
    print ('O is pressed')
    draw=False #If 'O' press enable drawing

  if draw:  # Enable robot coloring ONLY if initially no key is pressed or P is pressed ; stop colouring if O is pressed.
    translation = translationField.getSFVec3f() #Fetch robot location to color at that location
    x = int(height * (translation[1] + GROUND_X / 2) / GROUND_X) # Mapping robot translation in x axis to pixel value
    y = int(width * (translation[0] + GROUND_Y / 2) / GROUND_Y) # Mapping robot translation in y axis to pixel value
    
    # Remove 10 x 10 pixel at location x,y in the display image plane to see the base "Women in AI and Robotics" logo.
    display.fillOval( x, y, 10, 10)

  pass
