import serial
from PIL import Image, ImageTk
from io import BytesIO
import tkinter as tk
import numpy as np

# Open the UART connection on COM8
ser = serial.Serial('COM8', 9600)

start_str = "ACK CMD Capture done!\n\r"
end_str = "ACK CMD Tx complete!\n\r"
filter_str = "IRQH\n\r"

send_rate = 1000

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
        #Wheel dutie calculations here
        wheel_duties = bytes([np.uint8(0.3 * 100) for i in range(4)])
                
        # SEND IT
        ser.write(wheel_duties)
    
    data = ser.read()
    buffer += data
    if bytes(start_str, "utf-8") in buffer: # Start of image marker
        index = buffer.find(bytes(start_str, "utf-8"))
        buffer = buffer[index+len(start_str):]
        found = 1
    elif bytes(end_str, "utf-8") in buffer and found == 1: # End of image marker
        buffer=buffer[:buffer.index(bytes(end_str, "utf-8")):]

        display_image(buffer)
        
        found = 0
        buffer = b''
    elif bytes(filter_str, "utf-8") in buffer:
        index = buffer.find(bytes(start_str, "utf-8"))
        buffer = buffer[:index+1-len(filter_str)]
        
    window.update()