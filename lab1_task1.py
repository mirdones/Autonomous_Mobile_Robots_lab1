"""lab1_task1 controller."""

from controller import Robot
import math

WHEEL_DIST = 2.28
WHEEL_DIAMETER = 1.6
MAX_PHI = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftposition_sensor = robot.getDevice('left wheel sensor')
rightposition_sensor = robot.getDevice('right wheel sensor')


def initEncoders():
    global leftMotor, rightMotor, left_position_sensor, right_position_sensor, robot

    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(0)  # rad/s
    rightMotor.setVelocity(0)  # rad/s

    leftposition_sensor.enable(timestep)
    rightposition_sensor.enable(timestep)

    robot.step(timestep)


initEncoders()

left_position_sensor = leftposition_sensor.getValue()
right_position_sensor = rightposition_sensor.getValue()
previous_left_position_sensor = left_position_sensor
previous_right_position_sensor = right_position_sensor


def print_sensors():
    global left_position_sensor, right_position_sensor
    left_speed, right_speed = getSpeeds()

    print("left position sensor: {0:.4f}".format(left_position_sensor))
    print("right position sensor: {0:.4f}".format(right_position_sensor))
    print("left speed sensor: {0:.4f} revolutions per second".format(left_speed))
    print("right speed sensor: {0:.4f} revolutions per second\n".format(right_speed))


def step():
    global previous_left_position_sensor, previous_right_position_sensor, left_position_sensor, right_position_sensor

    previous_left_position_sensor = left_position_sensor
    previous_right_position_sensor = right_position_sensor

    robot.step(timestep)

    left_position_sensor = leftposition_sensor.getValue()
    right_position_sensor = rightposition_sensor.getValue()


def resetCounts():
    print("Simulator does not use ticks.\n")


def getCounts():
    print("Simulator does not use ticks.\n")


def getSpeeds():
    global previous_left_position_sensor, previous_right_position_sensor, left_position_sensor, right_position_sensor

    left_speed = (left_position_sensor - previous_left_position_sensor) / (timestep / 1000)  # rad/s
    right_speed = (right_position_sensor - previous_right_position_sensor) / (timestep / 1000)  # rad/s

    left_speed /= math.pi  # rps
    right_speed /= math.pi  # rps

    return left_speed, right_speed


def setSpeedsPWM(pwmLeft, pwmRight):
    print("Simulator does not use PWM.\n")


def setSpeedsRPS(rpsLeft, rpsRight):
    good_speed = True

    if math.fabs(rpsLeft) > MAX_PHI:
        print("Tried to set rpsLeft greater than MAX.")
        good_speed = False

    if math.fabs(rpsRight) > MAX_PHI:
        print("Tried to set rpsRight greater than MAX.")
        good_speed = False

    if good_speed:
        print("Left motor velocity: {0:.4f} rad/s".format(rpsLeft))
        print("Right motor velocity: {0:.4} rad/s\n".format(rpsRight))

        leftMotor.setVelocity(rpsLeft)
        rightMotor.setVelocity(rpsRight)
    else:
        print("Cannot complete movement\n")


def setSpeedsIPS(ipsLeft, ipsRight):
    PhiLeft = ipsLeft / (WHEEL_DIAMETER / 2)
    PhiRight = ipsRight / (WHEEL_DIAMETER / 2)

    setSpeedsRPS(PhiLeft, PhiRight)


def setSpeedsVW(V, W):
    R = V / W

    ipsLeft = W * (R - WHEEL_DIST / 2)
    ipsRight = W * (R + WHEEL_DIST / 2)

    setSpeedsIPS(ipsLeft, ipsRight)


def moveXV(X, V):
    movement_time = X / V
    print("movement time: {0:.3f} seconds".format(movement_time))

    number_timesteps = math.floor(movement_time / (timestep / 1000))
    print("number_timesteps to complete movement: {0}\n".format(number_timesteps))

    setSpeedsIPS(V, V)

    print_sensors()

    for n in range(number_timesteps):
        step()

    print_sensors()

    setSpeedsIPS(0, 0)


def turnRV(R, V):
    W = V / R
    setSpeedsVW(V, W)


# Main loop:
# - perform simulation steps until Webots is stopping the controller

initEncoders()

X = 10
V = 3
R = 10

print("move {0:.2f} inches at {1:.2f} inches/sec\n".format(X, V))
moveXV(10, 3)

print("move around a circle of radius {0:.2f} inches at a linear velocity of {1:.2f} inches/sec\n".format(R, V))
turnRV(10, 3)

while True:
    step()
