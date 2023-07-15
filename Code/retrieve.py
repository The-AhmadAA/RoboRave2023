# Cheese retrieval arm init and operation
from micromelon import *
import time
import math

# constant definitions
EXTEND_ANGLE = 75
STOW_ANGLE = -80
PREGRIP_POSITION = 60
GRIP_POSITION = 80
PREGRIP_STOW = 0

PRE_SCAN_TURN = 10
LEFT = -20
RIGHT = 20
INCREMENTS = 8
TURN_SPEED = 5

MIN_DIST = 4
MAX_DIST = 5

RETRIEVING = True


def extend_arm():
    # extend the arm to grabbing position
    Servos.right(EXTEND_ANGLE)


def stow_arm():
    # stow the arm once the cheese has been grabbed
    Servos.right(STOW_ANGLE)


def pregrip_pos():
    # position the gripper for pre grip, once the arm is deployed
    Servos.left(PREGRIP_POSITION)


def grip():
    # grip the cheese once in position
    Servos.left(GRIP_POSITION)

# Quinn's block


    # Params
ROVER_SIZE = 13
PRESENCE_DELTA = 5  # in cm, the sudden change to consider being an object
JUMP_FACTOR = 0.1  # Only consider if jump results in value change greater than this
SWEEP_ANGLE = 30
ROTATE_SPEED = 6
SWEEP_DISTANCE = (SWEEP_ANGLE / 360) * math.pi * ROVER_SIZE
TURN_TIME = SWEEP_DISTANCE / ROTATE_SPEED

# Detects if the cheese is still present or has been picked up.


def is_cheese_present():
    # Assume facing cheese-ish. Turn to sweep.
    Motors.turnDegrees(round(-SWEEP_ANGLE / 2))
    previousMeasure = Ultrasonic.read()
    newMeasure = previousMeasure
    Motors.write(ROTATE_SPEED, -ROTATE_SPEED)
    startTime = time.time()
    while time.time() - startTime < TURN_TIME:
        newMeasure = Ultrasonic.read()
        if abs(newMeasure - previousMeasure) > PRESENCE_DELTA and abs(newMeasure - previousMeasure) / previousMeasure > JUMP_FACTOR:
            Motors.write(0)
            return True
        previousMeasure = newMeasure
    Motors.write(0)
    return False


def scan_for_cheese(scan_limit) -> int:
    # scan left/right to find position of lowest Ultrasonic distance value
    smallest = 255
    bearing = 0
    turned = 0
    increment_count = 0

    # turn 20 degrees and find the smallest ultrasonic distance
    # reading, and the increment at which it was found
    while (turned < scan_limit):

        Motors.turnDegrees(scan_limit//INCREMENTS)
        turned += scan_limit//INCREMENTS
        if (smallest > Ultrasonic.read()):
            smallest = Ultrasonic.read()
            bearing = turned
            increment_count += 1
    # center on the cheese (lowest distance value)
    # ((scan_limit//INCREMENTS) *increment_count + 4))
    print("bearing = " + str(bearing))
    Motors.turnDegrees(-(scan_limit - bearing))

    adjust_distance(smallest)
    return bearing


def adjust_distance(smallest):
    distance = smallest
    if (distance < MIN_DIST or distance > MAX_DIST):
        while (True):
            # move the rover until the cheese is within distance limits
            if (distance < MIN_DIST):
                Motors.moveDistance(-1)  # (MIN_DIST - smallest))
            elif (distance > MAX_DIST):
                Motors.moveDistance(1)  # smallest - MAX_DIST)
            else:
                break
            distance = Ultrasonic.read()


def grab_cheese():

    # set leds to indicate cheese grabbing mode
    LEDs.writeAll([0, 255, 255])

    # configure arm for scan
    print("configuring arm for scan")
    stow_arm()
    pregrip_pos()

    # scan for the cheese
    print("Scanning for cheese")
    # scan left, scan right,
    print("scanning right")

    # turn left to scan past the cheese
    while True:
        Motors.turnDegrees(-PRE_SCAN_TURN)
        smallest_dir = scan_for_cheese(RIGHT)

        Motors.turnDegrees(smallest_dir-25)
        pregrip_pos()
        print("pregrip" + str(Servos.read()))
        time.sleep(1)
        extend_arm()
        time.sleep(1)
        # sweep to scoop cheese
        Motors.turnDegrees(smallest_dir*2, TURN_SPEED, 0, False)
        grip()
        print("gripping" + str(Servos.read()))
        time.sleep(1)
        stow_arm()
        # turn back to original bearing
        Motors.turnDegrees(-smallest_dir, TURN_SPEED, 0, False)
        if not is_cheese_present():
            Motors.turnDegrees(-SWEEP_ANGLE//2)
            break
        else:
            Motors.turnDegrees(-SWEEP_ANGLE//2)

    # NOTE: back up here?
    LEDs.writeAll([255, 255, 0])

    RETRIEVING = False


# for connecting to the rover when running this in isolation

# rc = RoverController()

# port = 83  # int(entry)
# rc.connectBLE(port)

# rc.startRover()

# # Pauses program until ready to start
# input("Press Enter to continue...")
# grab_cheese()

# rc.stopRover()
# rc.end()
