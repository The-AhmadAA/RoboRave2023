# Cheese retrieval arm init and operation
import micromelon as mc
from time import sleep

# constant definitions
EXTEND_ANGLE = 80
STOW_ANGLE = -80
PREGRIP_POSITION = 55
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
    mc.Servos.right(EXTEND_ANGLE)


def stow_arm():
    # stow the arm once the cheese has been grabbed
    mc.Servos.right(STOW_ANGLE)


def pregrip_pos():
    # position the gripper for pre grip, once the arm is deployed
    mc.Servos.left(PREGRIP_POSITION)


def grip():
    # grip the cheese once in position
    mc.Servos.left(GRIP_POSITION)


def scan_for_cheese(scan_limit) -> int:
    # scan left/right to find position of lowest Ultrasonic distance value
    smallest = 255
    bearing = 0
    turned = 0
    increment_count = 0

    # turn 20 degrees and find the smallest ultrasonic distance
    # reading, and the increment at which it was found
    while (turned < scan_limit):

        mc.Motors.turnDegrees(scan_limit//INCREMENTS)
        turned += scan_limit//INCREMENTS
        if (smallest > mc.Ultrasonic.read()):
            smallest = mc.Ultrasonic.read()
            bearing = turned
            increment_count += 1
    # center on the cheese (lowest distance value)
    # ((scan_limit//INCREMENTS) *increment_count + 4))
    mc.Motors.turnDegrees(-(turned))

    adjust_distance(smallest)
    return bearing


def adjust_distance(smallest):
    distance = smallest
    if (distance < MIN_DIST or distance > MAX_DIST):
        while (True):
            # move the rover until the cheese is within distance limits
            if (distance < MIN_DIST):
                mc.Motors.moveDistance(-1)  # (MIN_DIST - smallest))
            elif (distance > MAX_DIST):
                mc.Motors.moveDistance(1)  # smallest - MAX_DIST)
            else:
                break
            distance = mc.Ultrasonic.read()


def grab_cheese():

    # set leds to indicate cheese grabbing mode
    mc.LEDs.writeAll([0, 255, 255])

    # configure arm for scan
    print("configuring arm for scan")
    stow_arm()
    pregrip_pos()

    # scan for the cheese
    print("Scanning for cheese")
    # scan left, scan right,
    print("scanning right")

    # turn left to scan past the cheese
    mc.Motors.turnDegrees(-PRE_SCAN_TURN)
    smallest_dir = scan_for_cheese(RIGHT)

    print(f"Cheese found at bearing {smallest_dir}")

    mc.Motors.turnDegrees(smallest_dir - 25)
    pregrip_pos()
    print("pregrip" + str(mc.Servos.read()))
    sleep(1)
    extend_arm()
    sleep(1)
    # sweep to scoop cheese
    mc.Motors.turnDegrees(20, TURN_SPEED, 0, False)
    grip()
    print("gripping" + str(mc.Servos.read()))
    sleep(1)
    stow_arm()
    # turn back to original bearing
    mc.Motors.turnDegrees(-10, TURN_SPEED, 0, False)
    # NOTE: back up here?
    mc.LEDs.writeAll([255, 255, 0])

    RETRIEVING = False


# for connecting to the rover when running this in isolation

# rc = mc.RoverController()

# port = 83  # int(entry)
# rc.connectBLE(port)

# rc.startRover()

# # Pauses program until ready to start
# input("Press Enter to continue...")
# grab_cheese()

# rc.stopRover()
# rc.end()
