from micromelon import *
import retrieve
from time import *

# Constants

# Roborave cell size is slightly over 26.5 cm (including posts, approximately 27.7 cm)
CELL_SIZE = 27
MAX_DIST = 25  # distance to a wall
# Speed should have a magnitude no greater than 30
MOVE_SPEED = 25
MOVE_TIME = 1
TURN_RADIUS = 1
OFFSET = 0
TURN = 90  # changed this to 70 for a 90 deg turn due to some shonky wheel encoding/imu values?
LEFT = -1
RIGHT = 1
STRAIGHT = 0
BACK = 2

END_REACHED = False
# Functions

# Determines the change in angle required to correct the bearing


def correctBearing():
    rotation = IMU.readGyroAccum(2)
    offset = rotation % TURN
    if offset > (TURN / 2):
        offset -= TURN
    return 0  # round(offset)

# Attempts to recover the rover from a wall-strike situation


def recover(uncommanded_turn):
    '''
    Takes in the degrees of uncommanded turn and attempts to move the rover to a pre
    wall-strike configuration

    Arguments:
    uncommanded_turn(int): the number of degrees of uncommanded turn

    returns: None
    '''
    print(str(uncommanded_turn))
    Motors.moveDistance(-5)
    Motors.turnDegrees(uncommanded_turn)


# Removes dead ends from the path

def cullDeadEnds(path_taken):
    dead_ends = True
    while dead_ends:
        dead_ends = False
        for i in range(len(path_taken)):
            if path_taken[i] == BACK:
                dead_ends = True
                for j in range(1, i):
                    if path_taken[i - j] == STRAIGHT and path_taken[i + j] == STRAIGHT:
                        continue
                    elif path_taken[i - j] == RIGHT and path_taken[i + j] == LEFT:
                        continue
                    elif path_taken[i - j] == LEFT and path_taken[i + j] == RIGHT:
                        continue
                    else:
                        if path_taken[i - j] == LEFT and path_taken[i + j] == LEFT:
                            path_taken[i - j] = STRAIGHT
                        elif path_taken[i - j] == LEFT and path_taken[i + j] == STRAIGHT:
                            path_taken[i - j] = RIGHT
                        elif path_taken[i - j] == STRAIGHT and path_taken[i + j] == LEFT:
                            path_taken[i - j] = RIGHT
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

# maze specific turn function


def turn(direction):
    direction_word = {LEFT: "left", RIGHT: "right", BACK: "back"}
    print("turning " + direction_word[direction])
    # correction for arriving short of an intersection (due to IR sensor position)
    Motors.moveDistance(2)  # move into the intersection

    Motors.turnDegrees(direction * (TURN - OFFSET),
                       MOVE_SPEED//3, TURN_RADIUS, False)
    END_REACHED = forward()
    return direction

# maze specific forward motion


def forward():
    # while (Ultrasonic.read() > CELL_SIZE):
    print("going straight")
    while True:
        Motors.write(MOVE_SPEED)
        print("sleep started")
        sleep(MOVE_TIME)
        print("sleep finished")
        # print(str(rc.readAttribute()))
        if IR.readLeft() > MAX_DIST:
            print("left far")
            break
        if IR.readRight() > MAX_DIST:   # CELL_SIZE:
            print("right far")
            break
        if Ultrasonic.read() < MAX_DIST:
            print("front near")
            break
        if reachedEnd():
            Motors.write(0)
            return True

    Motors.write(0)
    return False
    # continue

# Determines whether the rover has reached the end square of the maze


def reachedEnd():
    # TODO: configure this for the actual maze!
    start = process_time_ns()
    while (process_time_ns() - start) < 50000000:
        # print("checking for endzone")
        colour_brightness = Colour.sensorSees(COLOURS.RED)
        if colour_brightness:
            print("endzone detected")
            return True

    return False


# Main control flow function

def main():
    LEDs.writeAll([0, 0, 255])
    retrieve.stow_arm()
    retrieve.pregrip_pos()

    path_taken = []
    # Employs a sort of depth-first search to 'hug the left wall', keeping
    # track of the path taken (flipping lefts and rights)
    while True:
        end_reached = END_REACHED
        if end_reached:
            break

        elif IR.readLeft() > MAX_DIST:  # opening on the left
            path_taken.append(turn(LEFT))

        elif IR.readRight() > MAX_DIST:  # opening on the right
            path_taken.append(turn(RIGHT))

        elif Ultrasonic.read() > MAX_DIST:  # corridor continues
            # Motors.turnDegrees(STRAIGHT * TURN + correctBearing(), MOVE_SPEED)
            # current_bearing = IMU.readGyro()[0]
            # Motors.moveDistance(CELL_SIZE, MOVE_SPEED)

            # # add in a recovery detection statement
            # new_bearing = IMU.readGyro()[0]
            # if abs(current_bearing - new_bearing) > 3:
            #     print("detected wall-strike?")
            #     # recover(int(new_bearing - current_bearing))

            # # go straight until a wall or end is reached
            end_reached = forward()
            path_taken.append(STRAIGHT)
            if end_reached:
                break

        elif Ultrasonic.read() < MAX_DIST and IR.readLeft() < MAX_DIST and IR.readRight() < MAX_DIST:
            # Motors.turnDegrees(BACK * TURN + correctBearing(), MOVE_SPEED)
            # Motors.moveDistance(CELL_SIZE, MOVE_SPEED)
            path_taken.append(turn(BACK))

        else:
            continue

    # Print statement helps show the rover is on track
    print("Reached goal!")
    retrieve.grab_cheese()
    while (retrieve.RETRIEVING):
        sleep(1)

    # TODO: reset position for start of return phase?

    # Remove dead-end routes from the path
    clean_path = cullDeadEnds(path_taken)
    # Reverses order of path taken to return correctly
    return_path = reversePath(clean_path)

    Motors.turnDegrees(BACK * TURN + correctBearing(), MOVE_SPEED)
    # Follows the path taken to return to start
    for i in return_path:
        Motors.moveDistance(CELL_SIZE, MOVE_SPEED)
        Motors.turnDegrees(i * TURN + correctBearing(), MOVE_SPEED)
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
