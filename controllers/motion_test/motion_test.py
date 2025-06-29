from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("rgb_camera")
camera.enable(timestep)


# Get rotational motor devices
left_motor = robot.getDevice('left_wheel_joint')
right_motor = robot.getDevice('right_wheel_joint')

# Set motor to velocity control mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Set target velocities (rad/s)
# To make a wide circle, left wheel moves faster than the right
left_speed = 4.0  # Adjust as needed for your robot's speed
right_speed = 2.0

left_motor.setVelocity(left_speed)
right_motor.setVelocity(right_speed)

# Run forever
while robot.step(timestep) != -1:
    pass  # Do nothing, just keep the motors running


