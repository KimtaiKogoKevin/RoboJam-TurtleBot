from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("rgb_camera")
camera.enable(timestep)

while robot.step(timestep) != -1:
    pass

