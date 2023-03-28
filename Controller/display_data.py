import serial
from PIL import Image, ImageTk
from io import BytesIO
import tkinter as tk

# Open the UART connection on COM8
ser = serial.Serial('COM8', 9600)

start_str = "ACK CMD capture done!\n\r"
end_str = "ACK CMD Captured!\n\r"

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
while True:
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
    window.update()