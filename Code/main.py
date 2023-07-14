from micromelon import *
import math
import time
from retrieve import *

# Constants

# -----
# Things to fiddle with for calibration:
# Roborave cell size is slightly over 26.5 cm (including posts, approximately 27.7 cm)
CELL_SIZE = 24  # if using sim use 36
# Rover length / width in cm
ROVER_SIZE = 13
# Speed should have a magnitude no greater than 30
MOVE_SPEED = 15
# The colour tolerance in detecting lines passed over
TOLERANCE = 10
# -----

# The initial angle reading to use as an offset later on
INIT_ANGLE = [0, 0, 0]
# Turning values
TURN = 70
LEFT = -1
RIGHT = 1
STRAIGHT = 0
BACK = 2

# Not so constants
# Stores whether the finish line has been reached.
REACHED_CHEESE = False
FINISHED = False
# The clearance to maintain when approaching walls, is updated throughout the run for accuracy
CLEARANCE = (CELL_SIZE - ROVER_SIZE) / 2

# Sim details
START_COLOUR = [160, 185, 137]  # FIXME: 'black' tape # [215, 190, 60]

END_COLOUR = [225, 195, 145]  # FIXME: red tape, change for green
# Functions

# Replaces IMU.readGyroAccum() to account for any initial accumulation prior to the start of the program.


def progGyroAccum(axis=None):
    if axis == None:
        reading = IMU.readGyroAccum()
        return [reading[0] - INIT_ANGLE[0], reading[1] - INIT_ANGLE[1], reading[2] - INIT_ANGLE[2]]
    elif axis >= 0 and axis <= 2:
        return IMU.readGyroAccum(axis) - INIT_ANGLE[axis]
    else:
        raise Exception(
            "Argument to progGyroAccum must be a number between 0 and 2")

# Determines the change in angle required to correct the bearing


def correctBearing():
    rotation = progGyroAccum(2)
    offset = rotation % TURN
    if offset > (TURN / 2):
        offset -= TURN
    return round(offset)

# Determines whether the rover is between two walls, and if so uses the difference in distance to center itself.


def selfCentreAngle(distance):
    if IR.readLeft() < CELL_SIZE and IR.readRight() < CELL_SIZE:
        # global CLEARANCE
        # CLEARANCE = (IR.readLeft() + IR.readRight() + 2 * CLEARANCE) / 4
        # dist_from_mid = abs(IR.readRight() - IR.readLeft()) / 2
        left = IR.readLeft()
        right = IR.readRight()
        dist_from_mid = abs(right - left) / 2
        dist_from_mid *= CELL_SIZE / (left + right + ROVER_SIZE)
        side = -1 * (not right > left) + \
            (right > left)
        if dist_from_mid == 0:
            return 0
        tan_theta = distance / dist_from_mid
        turn_angle = 90 - (math.atan(tan_theta) * 180 / math.pi)
        return side * round(turn_angle)
    return 0


def passesColour(colour):
    return Colour.sensorSees(colour, 0, TOLERANCE) \
        or Colour.sensorSees(colour, 1, TOLERANCE) \
        or Colour.sensorSees(colour, 2, TOLERANCE)

# Used to move the rover forward one cell


def moveForward(distance):
    Motors.write(MOVE_SPEED)
    start = time.time()
    while (time.time() - start < distance / MOVE_SPEED):
        # Stop when at the final cell
        if not REACHED_CHEESE:
            if reachedEnd():
                break
        else:
            reachedEnd()
        # Don't drive into a wall, stop if uncommanded rotation about x or y
        # or progGyroAccum(0) > TOLERANCE or progGyroAccum(1) > TOLERANCE:
        if Ultrasonic.read() < CLEARANCE:
            break
    Motors.write(0)
    delay(0.01)
    if Ultrasonic.read() < 1.5 * CLEARANCE:
        Motors.moveDistance(Ultrasonic.read() - 1.5 * CLEARANCE)

# Removes dead ends from the path


def cullDeadEnds(path_taken):
    dead_ends = True
    while dead_ends:
        print(len(path_taken), path_taken)
        dead_ends = False
        for i in range(len(path_taken)):
            # Dead end found when the rover had to double back on itself
            if path_taken[i] == BACK:
                dead_ends = True
                k = 0
                for j in range(1, i):
                    k = j
                    # Checks whether the moves are still backtracking
                    if path_taken[i - j] == STRAIGHT and path_taken[i + j] == STRAIGHT:
                        continue
                    elif path_taken[i - j] == RIGHT and path_taken[i + j] == LEFT:
                        continue
                    elif path_taken[i - j] == LEFT and path_taken[i + j] == RIGHT:
                        continue
                    else:
                        break
                # Replaces the original detour with the correct turn
                if path_taken[i - k] == LEFT and path_taken[i + k] == LEFT:
                    path_taken[i - k] = STRAIGHT
                elif path_taken[i - k] == LEFT and path_taken[i + k] == STRAIGHT:
                    path_taken[i - k] = RIGHT
                elif path_taken[i - k] == STRAIGHT and path_taken[i + k] == LEFT:
                    path_taken[i - k] = RIGHT
                # Removes the dead end from the route
                for _ in range(2 * k):
                    path_taken.pop(i - k + 1)
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
    global REACHED_CHEESE, FINISHED
    # -----
    # if not REACHED_CHEESE:
    #     colour_brightness = Colour.readSensor(CS.BRIGHT, 1)
    #     if colour_brightness > 180:
    #         REACHED_CHEESE = True
    #     return colour_brightness > 180
    # else:
    #     if passesColour(START_COLOUR):
    #         FINISHED = True
    #     return passesColour(START_COLOUR)
    # -----
    # Comment out everything in the above block for physical testing.
    if not REACHED_CHEESE and passesColour(END_COLOUR):  # COLOURS.GREEN):
        REACHED_CHEESE = True
        return True
    elif REACHED_CHEESE and passesColour(START_COLOUR):
        FINISHED = True
        return True
    else:
        return False

# Runs the logic for the 'hug-left-wall' algorithm and returns the decision


def searching():
    # Checks in turn: left, straight, right, and backward turns.
    if IR.readLeft() > CELL_SIZE:
        wall_boost = (IR.readRight() < CELL_SIZE) * \
            (CLEARANCE - IR.readRight())
        delay(0.1)
        Motors.turnDegrees(LEFT * TURN + correctBearing(), MOVE_SPEED)
        moveForward(CELL_SIZE + wall_boost)
        return LEFT
    elif Ultrasonic.read() > 0.75 * CELL_SIZE:
        Motors.turnDegrees(STRAIGHT * TURN + correctBearing() +
                           selfCentreAngle(CELL_SIZE), MOVE_SPEED)
        moveForward(CELL_SIZE)
        return STRAIGHT
    elif IR.readRight() > CELL_SIZE:
        wall_boost = (IR.readLeft() < CELL_SIZE) * (CLEARANCE - IR.readLeft())
        delay(0.1)
        Motors.turnDegrees(RIGHT * TURN + correctBearing(), MOVE_SPEED)
        moveForward(CELL_SIZE + wall_boost)
        return RIGHT
    else:
        if Ultrasonic.read() < CLEARANCE:
            Motors.moveDistance(Ultrasonic.read() - CLEARANCE)
        delay(0.1)
        Motors.turnDegrees(BACK * TURN, MOVE_SPEED)
        Motors.turnDegrees(correctBearing() +
                           selfCentreAngle(CELL_SIZE), MOVE_SPEED)
        moveForward(CELL_SIZE)
        return BACK

# Main control flow function


def main():
    stow_arm()
    pregrip_pos()

    path_taken = []
    # Employs a sort of depth-first search to 'hug the left wall', keeping
    # track of the path taken

    # Enters the maze
    moveForward(CELL_SIZE)
    path_taken.append(STRAIGHT)

    while not REACHED_CHEESE:
        if IMU.isFlipped():
            if Ultrasonic.read() < CELL_SIZE:
                Motors.moveDistance(-CELL_SIZE, MOVE_SPEED)
            else:
                Motors.moveDistance(CELL_SIZE, MOVE_SPEED)
        else:
            path_taken.append(searching())
    # Print statement helps show the rover is on track
    print("Reached goal!")

    # -----
    # back up to avoid case where cheese is close to the line
    Motors.moveDistance(-5)
    grab_cheese()
    # This is where the cheese pickup happens, replace this block.
    # Once cheese is picked up, will need to drive forward into the
    # middle of the cheese square, the rest should work from there
    # -----

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
        Motors.turnDegrees(i * TURN + correctBearing() +
                           selfCentreAngle(CELL_SIZE), MOVE_SPEED)
    print("Backtrack finished")
    while not FINISHED:
        searching()
    print("Returned to start!")


# ----------
# Setup of device - not part of the algorithm
# Nothing below here should need modifying

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
# Sets a reference orientation
INIT_ANGLE = IMU.readGyroAccum()
# Runs and times program
# start = time.time()
main()
# end = time.time()
# print(
#     f"The full run took {end - start} seconds to complete. This would result in {3 * (4 * 60 + start - end)} time points.")

rc.stopRover()
rc.end()
