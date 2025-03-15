import cv2
import apriltag

from libcamera import Transform
from picamera2 import Picamera2

picam2 = Picamera2()

transform = Transform(vflip=1, hflip=1)
config = picam2.create_still_configuration()
config["transform"] = transform
picam2.configure(config)

picam2.start()

picam2.capture_file("visual.png")
print("Image captured")

# Read the image using OpenCV
image = cv2.imread("visual.png")

# Convert the image to grayscale
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Save the grayscale image
cv2.imwrite("visual_gray_random.png", gray_image)
print("Grayscale image saved")
