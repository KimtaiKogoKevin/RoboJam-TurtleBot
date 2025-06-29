from controller import Supervisor

import math

# turtlebot geometry from the proto file
wheel_radius = 0.03575    # in metres
axle_length = 2 * 0.1165  # distance between wheels, in metres



robot = Supervisor()
robot_node = robot.getSelf()


timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("rgb_camera")
camera.enable(timestep)


# Get rotational motor devices
left_motor = robot.getDevice('left_wheel_joint')
right_motor = robot.getDevice('right_wheel_joint')

# Get wheel encoders for each motor
left_encoder = robot.getDevice("left_wheel_joint_sensor")
right_encoder = robot.getDevice("right_wheel_joint_sensor")
left_encoder.enable(timestep)
right_encoder.enable(timestep)

# Set motor to velocity control mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Set target velocities (rad/s)
# To make a wide circle, left wheel moves faster than the right
left_speed = 4.0  # Adjust as needed for your robot's speed
right_speed = 4.0

left_motor.setVelocity(left_speed)
right_motor.setVelocity(right_speed)


# using the supvervisor controller we have access to the ground truth
# This gets the robot's own Node reference
robot_node = robot.getSelf()

def get_ground_truth_pose(robot_node):
    # Get the translation (position)
    position = robot_node.getPosition()  # [x, y, z]
    x = position[0]
    y = position[2]  # Webots uses Y-up, so Z is forward

    # Get the orientation matrix (3×3 flat array)
    orientation = robot_node.getOrientation()
    # Forward direction is along negative z → heading from rotation matrix
    theta = math.atan2(orientation[2], orientation[8])  # atan2(-m13, m33)

    return x, y, theta

# Wait one simulation step to get valid initial encoder values
robot.step(timestep)
prev_left = left_encoder.getValue()
prev_right = right_encoder.getValue()

# Get the initial translation (position)
x, y, theta =  get_ground_truth_pose(robot_node)
print("Initial reading:")
print(f"Pose [odo]: x = {x:.3f} m, y = {y:.3f} m, θ = {math.degrees(theta):.1f}°")

step_count = 0

# Run forever
while robot.step(timestep) != -1:
    left = left_encoder.getValue()
    right = right_encoder.getValue()

    # Compute wheel displacements
    d_left = (left - prev_left) * wheel_radius
    d_right = (right - prev_right) * wheel_radius

    # Average distance
    d_center = (d_left + d_right) / 2

    # Approximate change in heading
    d_theta = (d_right - d_left) / axle_length

    # Update pose
    theta += d_theta

    # Midpoint heading for better approximation
    theta_mid = theta + d_theta / 2
    x += d_center * math.cos(theta_mid)
    y += d_center * math.sin(theta_mid)

    x_gt, y_gt, theta_gt =  get_ground_truth_pose(robot_node)

    # Store for next loop
    prev_left = left
    prev_right = right


    step_count += 1
    if step_count % 1 == 0:  # Print every ~20 steps
        print(f"Δd = {d_center:.4f} m, Δθ = {d_theta:.4f} rad")
        print(f"Pose [odo]: x = {x:.3f} m, y = {y:.3f} m, θ = {math.degrees(theta):.1f}°")
        print(f"Pose [GT] : x = {x_gt:.3f} m, y = {y_gt:.3f} m, θ = {math.degrees(theta_gt):.1f}°")


