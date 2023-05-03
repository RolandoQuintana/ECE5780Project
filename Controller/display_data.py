import serial
from PIL import Image, ImageTk
from io import BytesIO
import tkinter as tk
import numpy as np
import inputs
import math

# Open the UART connection on COM8
ser = serial.Serial('COM5', 9600)

# For tuning 
ZERO_TOLERANCE = 0.15
STATIC_WHEEL_MULTIPLIER = 1
DYNAMIC_WHEEL_MULTIPLIER = 1.3
# Don't mess with this one, is good
TOTALSTEPS = 2**16 / 2
x, y, z = 0, 0, 0

# Check if theres even a controller connected
try:
    inputs.get_gamepad()
except Exception as e:
    print(e)
    exit(0)
    
    

def calculate_duties(x, y, z):
    if z > .5 or z < -.5:
        if z < 0:
            speed = min(z*-1, 1)
            return -speed, -speed, speed, speed
        else:
            speed = min(z, 1)
            return speed, speed, -speed, -speed
    else:
        z = 0
        # this will be the duty cycle for the set of wheels that change direction in a quadrant
        m = math.sqrt(x**2 + y**2) * (math.atan(abs(y/(x if x != 0 else 0.001))) - math.pi/4) * DYNAMIC_WHEEL_MULTIPLIER
        # duty cycle for set of wheels that stay the same in a quadrant
        ma = math.sqrt(x**2 + y**2) * STATIC_WHEEL_MULTIPLIER
        # clamp both values
        m = min(m, 1)
        ma = min(ma, 1)
        # right side of plane
        if x > ZERO_TOLERANCE:
            # first quadrant
            if y > ZERO_TOLERANCE:
                return m, ma, ma, m
            # fourth quadrant
            elif y < -ZERO_TOLERANCE:
                return -ma, -m, -m, -ma
            # dead zone, only go right
            else:
                return -ma, ma, ma, -ma
        # left side of plane
        elif x < -ZERO_TOLERANCE:
            # second quadrant
            if y > ZERO_TOLERANCE:
                return ma, m, m, ma
            # third quadrant
            elif y < -ZERO_TOLERANCE:
                return -m, -ma, -ma, -m
            # dead zone, only go left
            else:
                return ma, -ma, -ma, ma
        # horizontal deadzone
        else:
            # go up
            if y > ZERO_TOLERANCE:
                return ma, ma, ma, ma
            # go down
            elif y < -ZERO_TOLERANCE:
                return -ma, -ma, -ma, -ma
            # don't move
            else:
                return 0, 0, 0, 0
  

start_str = "ACK CMD Capture done!\n\r"
end_str = "ACK CMD Tx complete!\n\r"
filter_str = "IRQH\n\r"

send_rate = 50

# Helper for turning those binary numbers back into signed 8 bit ints
def printWheelSpeeds(wheel_duties):
    return np.int8(wheel_duties[0]), np.int8(wheel_duties[1]), np.int8(wheel_duties[2]), np.int8(wheel_duties[3])
        

# Create a Tkinter window
window = tk.Tk()

# Set the title and window size
window.title("Image Viewer")
        
# Create a label for displaying the image
label = tk.Label(window)
label.pack()

# Function for displaying the image
def display_image(buffer):
    # Decode the JPEG image using Pillow and display it
    image = Image.open(BytesIO(buffer))
    
    # Scale up the image using Pillow's resize method
    scale = 5
    new_size = (image.width * scale, image.height * scale)
    image = image.resize(new_size)#, Image.ANTIALIAS)
    
    window.geometry("{}x{}".format(image.width, image.height))

    # Create a Tkinter PhotoImage from the PIL image
    photo = ImageTk.PhotoImage(image)

    # Update the label's image with the new one
    label.config(image=photo)
    label.image = photo
    
# Read incoming data from the UART connection and buffer it
buffer = b''
found = 0

wheel_counter = 0

while True:
    wheel_counter += 1
    wheel_counter = wheel_counter % send_rate
    if wheel_counter == 0:
        # try here because events will throw an error if controller gets disconnected
        try:
            events = inputs.get_gamepad()
            for event in events:
                # only get the left joystick events
                if event.code == inputs.ABSOLUTE_AXES[0][1]:
                    x = event.state / TOTALSTEPS
                if event.code == inputs.ABSOLUTE_AXES[1][1]:
                    y = event.state / TOTALSTEPS
                if event.code == inputs.ABSOLUTE_AXES[3][1]:
                    z = event.state / TOTALSTEPS
                    
                # Turn the floats into signed 8 bit ints, then turn into byte array
                wheel_duties = bytes([np.uint8(i * 100) for i in calculate_duties(x,y,z)])
                
                # SEND IT
                ser.write(wheel_duties)
                
                # For debugging purposes
                print(printWheelSpeeds(wheel_duties))

        except Exception as e:
            print("Controller disconnected" + str(e))
            exit(0)


    data = ser.read()
    buffer += data
    if bytes(start_str, "utf-8") in buffer: # Start of image marker
        index = buffer.find(bytes(start_str, "utf-8"))
        buffer = buffer[index+len(start_str):]
        found = 1
    elif bytes(end_str, "utf-8") in buffer and found == 1: # End of image marker
        buffer=buffer[:buffer.index(bytes(end_str, "utf-8")):]
        
        try:
            display_image(buffer)
        except Exception as e:
            print("ERROR IMAGE WHOAAAAAAA")
        
        found = 0
        buffer = b''
    elif bytes(filter_str, "utf-8") in buffer:
        index = buffer.find(bytes(start_str, "utf-8"))
        buffer = buffer[:index+1-len(filter_str)]
        
    window.update()