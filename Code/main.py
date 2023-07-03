from micromelon import *

# Constants

# Roborave cell size is slightly over 26.5 cm (including posts, approximately 27.7 cm)
CELL_SIZE = 36
# Speed should have a magnitude no greater than 30
MOVE_SPEED = 25
TURN = 90
LEFT = -1
RIGHT = 1
STRAIGHT = 0
BACK = 2

# Functions

# Determines the change in angle required to correct the bearing
def correct_bearing():
    rotation = IMU.readGyroAccum(2)
    offset = rotation % TURN
    if offset > (TURN / 2):
        offset -= TURN
    return round(offset)

# Determines whether the rover has reached the end square of the maze
def reached_end():
    colour_brightness = Colour.readSensor(CS.BRIGHT, 1)
    return colour_brightness > 180

# Main control flow function

def main():
    path_taken = []
    # Employs a sort of depth-first search to 'hug the left wall', keeping
    # track of the path taken (flipping lefts and rights)
    while True:
        if IR.readLeft() > CELL_SIZE:
            Motors.turnDegrees(LEFT * TURN + correct_bearing(), MOVE_SPEED)
            Motors.moveDistance(CELL_SIZE, MOVE_SPEED)
            path_taken.append(RIGHT)
        elif Ultrasonic.read() > CELL_SIZE:
            Motors.turnDegrees(STRAIGHT * TURN + correct_bearing(), MOVE_SPEED)
            Motors.moveDistance(CELL_SIZE, MOVE_SPEED)
            path_taken.append(STRAIGHT)
        elif IR.readRight() > CELL_SIZE:
            Motors.turnDegrees(RIGHT * TURN + correct_bearing(), MOVE_SPEED)
            Motors.moveDistance(CELL_SIZE, MOVE_SPEED)
            path_taken.append(LEFT)
        else:
            Motors.turnDegrees(BACK * TURN + correct_bearing(), MOVE_SPEED)
            Motors.moveDistance(CELL_SIZE, MOVE_SPEED)
            path_taken.append(BACK)
        Motors.write(0, 0, 0.1)
        if reached_end():
            break
    # Print statement helps show the rover is on track
    print("Reached goal!")
    # Reverses order of path taken to return correctly
    path_taken.reverse()
    Motors.turnDegrees(BACK * TURN + correct_bearing(), MOVE_SPEED)
    # Follows the path taken to return to start
    for i in path_taken:
        Motors.moveDistance(CELL_SIZE, MOVE_SPEED)
        Motors.turnDegrees(i * TURN + correct_bearing(), MOVE_SPEED)
        Motors.write(0, 0, 0.1)
    print("Returned to start!")

# Setup of device - not part of the algorithm

rc = RoverController()

# Sets up the connection type
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

# Connects to the rover
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

# Pauses program until ready to start
input("Press Enter to continue...")
main()

rc.stopRover()
rc.end()
