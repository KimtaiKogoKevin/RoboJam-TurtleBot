from controller import Robot
import numpy as np
import cv2

# --- HELPER FUNCTION FOR FIELD ISOLATION ---
def isolate_field_area(bgr_image):
    """
    Isolates the green soccer field from an image.

    Args:
        bgr_image: The input image in BGR format.

    Returns:
        A tuple containing:
        - field_mask: A binary mask where the field is white and everything else is black.
        - field_only_image: The original image with non-field areas blacked out.
    """
    # 1. Convert the image from BGR to HSV color space
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

    # 2. Define the range for the color green in HSV
    # These values might need tuning for your specific simulation's lighting!
    # A good starting point for typical "soccer pitch" green.
    lower_green = np.array([35, 40, 40])  # Lower bound for Hue, Saturation, Value
    upper_green = np.array([85, 255, 255]) # Upper bound for Hue, Saturation, Value

    # 3. Create a binary mask.
    # Pixels in hsv_image within the green range will be white (255), others will be black (0).
    initial_mask = cv2.inRange(hsv_image, lower_green, upper_green)

    # 4. (Optional but recommended) Clean the mask to fill in holes (like white lines).
    # A "closing" operation is a dilation followed by an erosion.
    # It's great for closing small holes inside foreground objects.
    kernel = np.ones((7, 7), np.uint8) # The size of the kernel determines how large of a hole to fill
    field_mask = cv2.morphologyEx(initial_mask, cv2.MORPH_CLOSE, kernel)

    # 5. Apply the mask to the original image to "cut out" the field
    # The bitwise_and operation only keeps pixels where the mask is white.
    field_only_image = cv2.bitwise_and(bgr_image, bgr_image, mask=field_mask)

    return field_mask, field_only_image


# --- YOUR MAIN SCRIPT ---

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- Device Initialization ---
camera = robot.getDevice("rgb_camera")
camera.enable(timestep)
gps = robot.getDevice("gps")
gps.enable(timestep)


# --- GRID SETUP (Your existing code) ---
FIELD_LENGTH = 9.0
FIELD_WIDTH = 6.0
GRID_RESOLUTION = 0.1
GRID_ROWS = int(FIELD_LENGTH / GRID_RESOLUTION)
GRID_COLS = int(FIELD_WIDTH / GRID_RESOLUTION)
grid_image = np.ones((GRID_ROWS, GRID_COLS, 3), dtype=np.uint8) * 255
for i in range(0, GRID_ROWS, 10):
    cv2.line(grid_image, (0, i), (GRID_COLS, i), (200, 200, 200), 1)
for j in range(0, GRID_COLS, 10):
    cv2.line(grid_image, (j, 0), (j, GRID_ROWS), (200, 200, 200), 1)
grid_image = cv2.flip(grid_image, 0)
cv2.imwrite("grid_overlay.png", grid_image)

# --- INITIAL IMAGE CAPTURE (Your existing code) ---
# Wait for a few timesteps for the camera to initialize
for _ in range(5):
    robot.step(timestep)

width = camera.getWidth()
height = camera.getHeight()
image = camera.getImage()
image_array = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
bgr_image = image_array[:, :, :3]
cv2.imwrite("initial_frame.png", bgr_image)
print("Saved initial camera frame as initial_frame.png")


# --- MAIN ROBOT LOOP ---
while robot.step(timestep) != -1:
    # --- Get Camera Image ---
    image = camera.getImage()
    image_array = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
    bgr_image = image_array[:, :, :3]

    # --- >>> NEW FIELD ISOLATION LOGIC <<< ---
    # Call our new function to get the field mask and the isolated field image
    field_mask, field_only_image = isolate_field_area(bgr_image)

    # Save the results for inspection
    cv2.imwrite("field_mask.png", field_mask)
    cv2.imwrite("field_only_image.png", field_only_image)
    # --- END OF NEW LOGIC ---

    # --- LINE DETECTION (can now be done on the cleaner mask or image) ---
    # It's often better to detect lines on the original grayscale image or a clean mask.
    # Let's use the original logic for now.
    gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
    _, binary_lines = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    lines = cv2.HoughLinesP(binary_lines, 1, np.pi/180, threshold=50, minLineLength=30, maxLineGap=10)

    line_image = bgr_image.copy()
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 2) # Changed to red for visibility
    cv2.imwrite("detected_lines.png", line_image)


    # --- POSITION ESTIMATION (Your existing code) ---
    position = gps.getValues()
    robot_x = position[0]
    robot_z = position[2]

    # Note: Webots Z is often vertical, check your setup. Assuming X and Z are ground plane.
    grid_x = int((robot_x + FIELD_LENGTH / 2) / GRID_RESOLUTION)
    grid_y = int((robot_z + FIELD_WIDTH / 2) / GRID_RESOLUTION)

    grid_copy = grid_image.copy()
    # Check bounds before drawing
    if 0 <= grid_x < GRID_ROWS and 0 <= grid_y < GRID_COLS:
        # Note the coordinate swap for opencv drawing (y, x) -> (col, row)
        cv2.circle(grid_copy, (grid_y, grid_x), 2, (0, 0, 255), -1)

    cv2.imwrite("robot_position_on_grid.png", grid_copy)

    # print(f"Robot at world: ({robot_x:.2f}, {robot_z:.2f}) → Grid: ({grid_x}, {grid_y})")