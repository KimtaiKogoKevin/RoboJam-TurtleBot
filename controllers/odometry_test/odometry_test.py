"""odometry_test controller."""

from controller import Supervisor
from drive_kinematics import *

import math

# turtlebot geometry from the proto file
#   name "left_wheel"
#   boundingObject Cylinder {
#   radius 0.035750000000000004

#   translation 0.000000 -0.116500 0.040200

wheel_radius = 0.03575    # in metres
pulses_per_turn = 508.8   # https://github.com/turtlebot/turtlebot4/issues/126
axle_length = 2 * 0.1165  # distance between wheels, in metres

r = wheel_radius    # radius of the wheels of the e-puck robot: 20.5mm 
l = axle_length    # distance between the wheels of the e-puck robot: 52mm


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
right_speed = 2.0

left_motor.setVelocity(left_speed)
right_motor.setVelocity(right_speed)


# using the supvervisor controller we have access to the ground truth
# This gets the robot's own Node reference
robot_node = robot.getSelf()

def get_ground_truth_pose(robot_node):
    # Get the translation (position)
    position = robot_node.getPosition()  # [x, y, z]
    x = position[0]
    y = position[1]    

    # Get the orientation matrix (3×3 flat/robo array)
    # https://cyberbotics.com/doc/reference/supervisor#wb_supervisor_node_get_orientation
    orientation = robot_node.getOrientation()

    # forward direction is yaw (rotation around z-axis)
    # R = [ R11  R12  R13 ] : [ R[0] R[1] R[2] ]
    #     [ R21  R22  R23 ] : [ R[3] R[4] R[5] ]
    #     [ R31  R32  R33 ] : [ R[6] R[7] R[8] ]

    # yaw: \theta_z = atan2(r_21, r_11)
    theta = math.atan2(orientation[3], orientation[0])  # atan2(-m13, m33)

    return x, y, theta

# Wait one simulation step to get valid initial encoder values
robot.step(timestep)
prev_left = left_encoder.getValue()
prev_right = right_encoder.getValue()

# Get the initial translation (position)
x, y, theta =  get_ground_truth_pose(robot_node)
print("Initial reading:")
print(f"Pose [odo]: x = {x:.3f} m, y = {y:.3f} m, θ = {math.degrees(theta):.1f}°")


x_old, y_old, theta_old = x, y, theta
step_count = 0

# Run forever
while robot.step(timestep) != -1:
    # wheel position sensor in webots returns radians
    # https://cyberbotics.com/doc/reference/positionsensor#positionsensor
    left = left_encoder.getValue()
    right = right_encoder.getValue()

    delta_t = timestep * 1e-3      # time step in seconds

    wl = (left - prev_left)/delta_t
    wr = (right - prev_right)/delta_t

    u, w = get_robot_speeds(wl, wr, r, l)

    x, y, theta = get_robot_pose(u, w, x_old, y_old, theta_old, delta_t
                                 )

    print(f"The new robot pose is: {x:.3f} m, {y:.3f} m, {theta*180/np.pi:.3f} deg.")


    print(f"Robot linear speed  = {u} m/s")
    print(f"Robot angular speed = {w} rad/s")


    print(f'Left wheel speed  = {wl} rad/s. ({left} : {prev_left} radians)')
    print(f'Right wheel speed = {wr} rad/s. ({right} : {prev_right} radians)')

    x_gt, y_gt, theta_gt =  get_ground_truth_pose(robot_node)

    # Store for next loop
    prev_left = left
    prev_right = right
    x_old, y_old, theta_old = x, y, theta


    step_count += 1
    if step_count % 1 == 0:  # Print every ~20 steps
        print(f"time: {robot.getTime()}")
        print(f"robot position: {robot_node.getPosition()}")

        print(f"Pose [odo]: x = {x:.3f} m, y = {y:.3f} m, θ = {math.degrees(theta):.1f}°")
        print(f"Pose [GT] : x = {x_gt:.3f} m, y = {y_gt:.3f} m, θ = {math.degrees(theta_gt):.1f}°")


