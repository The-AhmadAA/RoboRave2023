from micromelon import *

# Constants

CELL_SIZE = 36
MOVE_SPEED = 20
RIGHT_TURN = 90
FULL_TURN = 2 * RIGHT_TURN

# Functions

def correct_bearing():
    rotation = IMU.readGyroAccum(2)
    offset = rotation % RIGHT_TURN
    if offset > (RIGHT_TURN / 2):
        offset -= RIGHT_TURN
    Motors.turnDegrees(-round(offset), MOVE_SPEED)

# Main control flow function

def main():
    while True:
        if IR.readLeft() > CELL_SIZE:
            Motors.turnDegrees(-RIGHT_TURN, MOVE_SPEED)
            correct_bearing()
            Motors.moveDistance(CELL_SIZE, MOVE_SPEED)
        elif Ultrasonic.read() > CELL_SIZE:
            Motors.moveDistance(CELL_SIZE, MOVE_SPEED)
        elif IR.readRight() > CELL_SIZE:
            Motors.turnDegrees(RIGHT_TURN, MOVE_SPEED)
            correct_bearing()
            Motors.moveDistance(CELL_SIZE, MOVE_SPEED)
        else:
            Motors.turnDegrees(FULL_TURN, MOVE_SPEED)
            correct_bearing()
            Motors.moveDistance(CELL_SIZE, MOVE_SPEED)
        LEDs.writeAll(Colour.random())
        Motors.write(0, 0, 0.1)

# Setup of device

rc = RoverController()

usingSim = False
print("Are you using:", "A) a bot; or", "B) simulation?", sep="\n")
while True:
    entry = input()
    if entry.upper() == "A":
        usingSim = False
        break
    elif entry.upper() == "B":
        usingSim = True
        break
    else:
        print("Invalid input, please enter \"A\" or \"B\".")

print("Please enter your rover number:")
while True:
    entry = input()
    if entry.isnumeric():
        port = int(entry)
        if usingSim:
            try:
                rc.connectIP("127.0.0.1", port)
            except:
                print("Invalid port provided. Please try again.")
            else:
                break
        else:
            try:
                rc.connectBLE(port)
            except:
                print("Device is not ready for use. Please try another bot.")
            else:
                break
    else:
        print("Please enter a numerical value.")

rc.startRover()

input("Press Enter to continue...")
main()

rc.stopRover()
rc.end()
