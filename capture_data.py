import os
import serial
import struct
from PIL import Image, ImageOps
import matplotlib.pyplot as plt
from IPython.display import display, clear_output
import time
import keyboard

# Constants
cameraWidth = 96
cameraHeight = 96
cameraBytesPerPixel = 1
bytesPerFrame = cameraWidth * cameraHeight * cameraBytesPerPixel

# Serial setup (adjust COM port and baud rate as needed)
ser = serial.Serial('COM5', 921600, timeout=1)

# Create a blank image to display the camera feed
image = Image.new("L", (cameraWidth, cameraHeight))  # "L" mode is for grayscale

# Create a folder to save the images if it doesn't exist
output_folder = "raw_images"
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

# Counter for naming the images
image_counter = 0

# Function to read and display camera data in Jupyter and save when "Enter" is pressed
def read_frame_and_save():
    global image, image_counter
    
    # Wait for the start of the frame (0x55, 0xAA)
    while True:
        header = ser.read(1)
        if header == b'\x55':  # Look for start byte
            second_byte = ser.read(1)
            if second_byte == b'\xAA':  # Check for second start byte
                break  # Valid frame header found, exit loop
    
    # Try to read the frame buffer
    frameBuffer = ser.read(bytesPerFrame)
    
    # Ensure we received enough data
    if len(frameBuffer) == bytesPerFrame:
        # Update the image with the new data
        image.putdata(frameBuffer)
        
        # Display the image
        clear_output(wait=True)  # Clear previous output
        plt.imshow(ImageOps.flip(image), cmap='gray')  # Flip to correct orientation
        plt.axis('off')  # Hide axes
        display(plt.gcf())  # Display the current figure

        # Save the image when "Enter" is pressed
        if keyboard.is_pressed("enter"):
            image_path = os.path.join(output_folder, f"image_{image_counter}.png")
            image.save(image_path)
            print(f"Image saved: {image_path}")
            image_counter += 1

# Main loop to capture images on "Enter" and quit on "x"
try:
    print("Press 'Enter' to capture an image and 'x' to exit.")
    while True:
        if keyboard.is_pressed("x"):
            print("Exiting program.")
            break
        else:
            read_frame_and_save()
        time.sleep(0.1)  # Adjust the sleep time as needed

except KeyboardInterrupt:
    # Close the serial connection when interrupted
    ser.close()