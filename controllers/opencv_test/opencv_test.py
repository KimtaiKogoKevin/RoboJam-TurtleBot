from controller import Robot
import numpy as np
import cv2

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("rgb_camera")
camera.enable(timestep)

# Wait for a few timesteps so the camera has time to capture
for _ in range(5):
    robot.step(timestep)


# Get image dimensions
width = camera.getWidth()
height = camera.getHeight()

# Get the raw image (as bytes)
image = camera.getImage()

# Convert to numpy array (Webots returns BGRA format)
image_array = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))

# Drop the alpha channel (OpenCV expects BGR)
bgr_image = image_array[:, :, :3]

# Save the image
cv2.imwrite("frame.png", bgr_image)
print("Saved image as frame.png")

while robot.step(timestep) != -1:
    pass

