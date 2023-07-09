from micromelon import *
import math
import time

# Constants

# The initial angle reading to use as an offset later on
INIT_ANGLE = [0, 0, 0]
# Roborave cell size is slightly over 26.5 cm (including posts, approximately 27.7 cm)
CELL_SIZE = 36
CLEARANCE = 15
# Speed should have a magnitude no greater than 30
MOVE_SPEED = 25
TURN = 90
LEFT = -1
RIGHT = 1
STRAIGHT = 0
BACK = 2
# Stores whether the finish line has been reached.
REACHED_CHEESE = False
FINISHED = False

# Functions

# Replaces IMU.readGyroAccum() to account for any initial accumulation prior to the start of the program.
def progGyroAccum(axis=None):
    if axis == None:
        reading = IMU.readGyroAccum()
        return [reading[0] - INIT_ANGLE[0], reading[1] - INIT_ANGLE[1], reading[2] - INIT_ANGLE[2]]
    elif axis >= 0 and axis <= 2:
        return IMU.readGyroAccum(axis) - INIT_ANGLE[axis]
    else:
        raise Exception("Argument to IMU.readGyroAccum must be a number between 0 and 2")

# Determines the change in angle required to correct the bearing
def correctBearing():
    rotation = progGyroAccum(2)
    offset = rotation % TURN
    if offset > (TURN / 2):
        offset -= TURN
    return round(offset)

# Determines whether the rover is between two walls, and if so uses the difference in distance to center itself.
def selfCentreAngle():
    if IR.readLeft() < CELL_SIZE and IR.readRight() < CELL_SIZE:
        dist_from_mid = abs(IR.readRight() - IR.readLeft()) / 2
        side = -1 * (not IR.readRight() > IR.readLeft()) + (IR.readRight() > IR.readLeft())
        if dist_from_mid == 0:
            return 0
        tan_theta = CELL_SIZE / dist_from_mid
        turn_angle = 90 - (math.atan(tan_theta) * 180 / math.pi)
        return side * round(turn_angle)
    return 0

# Used to move the rover forward one cell
def moveForward(distance):
    # Motors.moveDistance(distance, MOVE_SPEED)
    Motors.write(MOVE_SPEED)
    start = time.time()
    while (time.time() - start < distance / MOVE_SPEED):
        if COLOURS.GREEN in Colour.readAllSensors(CS.ALL):
            REACHED_CHEESE = True
            break
        elif COLOURS.BLACK in Colour.readAllSensors(CS.ALL):
            FINISHED = True
    Motors.write(0)
    delay(0.01)
    if Ultrasonic.read() < CLEARANCE:
        Motors.moveDistance(Ultrasonic.read() - CLEARANCE)

# Removes dead ends from the path
def cullDeadEnds(path_taken):
    dead_ends = True
    while dead_ends:
        dead_ends = False
        for i in range(len(path_taken)):
            # Dead end found when the rover had to double back on itself
            if path_taken[i] == BACK:
                dead_ends = True
                for j in range(1, i):
                    # Checks whether the moves are still backtracking
                    if path_taken[i - j] == STRAIGHT and path_taken[i + j] == STRAIGHT:
                        continue
                    elif path_taken[i - j] == RIGHT and path_taken[i + j] == LEFT:
                        continue
                    elif path_taken[i - j] == LEFT and path_taken[i + j] == RIGHT:
                        continue
                    else:
                        # Replaces the original detour with the correct turn
                        if path_taken[i - j] == LEFT and path_taken[i + j] == LEFT:
                            path_taken[i - j] = STRAIGHT
                        elif path_taken[i - j] == LEFT and path_taken[i + j] == STRAIGHT:
                            path_taken[i - j] = RIGHT
                        elif path_taken[i - j] == STRAIGHT and path_taken[i + j] == LEFT:
                            path_taken[i - j] = RIGHT
                        # Removes the dead end from the route
                        for _ in range(2 * j):
                            path_taken.pop(i - j + 1)
                        break
            if dead_ends:
                break
    return path_taken

# Reverses the path such that it can be followed from end to beginning
def reversePath(path_taken):
    path_taken.reverse()
    for i in range(len(path_taken)):
        if path_taken[i] == LEFT:
            path_taken[i] = RIGHT
        elif path_taken[i] == RIGHT:
            path_taken[i] = LEFT
    return path_taken

# Determines whether the rover has reached the end square of the maze
def reachedEnd():
    colour_brightness = Colour.readSensor(CS.BRIGHT, 1)
    return colour_brightness > 180 or REACHED_CHEESE

# Main control flow function

def main():
    path_taken = []
    # Employs a sort of depth-first search to 'hug the left wall', keeping
    # track of the path taken (flipping lefts and rights)
    while True:
        # Checks in turn: left, straight, right, and backward turns.
        if IR.readLeft() > CELL_SIZE:
            delay(0.1)
            Motors.turnDegrees(LEFT * TURN + correctBearing(), MOVE_SPEED)
            moveForward(CELL_SIZE)
            path_taken.append(LEFT)
        elif Ultrasonic.read() > CELL_SIZE:
            Motors.turnDegrees(STRAIGHT * TURN + correctBearing() + selfCentreAngle(), MOVE_SPEED)
            moveForward(CELL_SIZE)
            path_taken.append(STRAIGHT)
        elif IR.readRight() > CELL_SIZE:
            delay(0.1)
            Motors.turnDegrees(RIGHT * TURN + correctBearing(), MOVE_SPEED)
            moveForward(CELL_SIZE)
            path_taken.append(RIGHT)
        else:
            delay(0.1)
            Motors.turnDegrees(BACK * TURN, MOVE_SPEED)
            Motors.turnDegrees(correctBearing() + selfCentreAngle(), MOVE_SPEED)
            moveForward(CELL_SIZE)
            path_taken.append(BACK)
        if reachedEnd():
            break
    # Print statement helps show the rover is on track
    print("Reached goal!")
    # Remove dead-end routes from the path
    clean_path = cullDeadEnds(path_taken)
    print("Path cleansed")
    # Reverses order of path taken to return correctly
    return_path = reversePath(clean_path)
    print("Path reversed")

    Motors.turnDegrees(BACK * TURN + correctBearing(), MOVE_SPEED)
    # Follows the path taken to return to start
    for i in return_path:
        moveForward(CELL_SIZE)
        delay(0.1)
        centering_angle = 0
        if i == STRAIGHT:
            centering_angle = selfCentreAngle()
        elif i == BACK:
            centering_angle = -selfCentreAngle()
        Motors.turnDegrees(i * TURN + correctBearing() + centering_angle, MOVE_SPEED)
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
start = time.time()
INIT_ANGLE = IMU.readGyroAccum()
main()
end = time.time()
print(f"The full run took {end - start} seconds to complete. This would result in {3 * (4 * 60 + start - end)} time points.")

rc.stopRover()
rc.end()
