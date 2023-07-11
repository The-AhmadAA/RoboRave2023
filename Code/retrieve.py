# Cheese retrieval arm init and operation 
import micromelon as mc

# constant definitions
EXTEND_ANGLE = 180
STOW_ANGLE = 0
PREGRIP_POSITION = 90
GRIP_POSITION = 120
PREGRIP_STOW = 0

LEFT = -20
RIGHT = 20
INCREMENTS = 8

def extend_arm():
   # extend the arm to grabbing position 
   # FIXME: servo functions are not being recognised?
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
   # scan left/right to find position of lowest US distance
    smallest = 255
    bearing = 0
    turned = 0

    # turn 20 degrees and find the smalled ultrasonic distance 
    # reading., and the increment at which it was found
    while(turned < scan_limit):

        mc.Motors.turnDegrees(scan_limit//INCREMENTS)
        turned += scan_limit//INCREMENTS
        if (smallest > mc.Ultrasonic.read()):
            smallest = mc.Ultrasonic.read()
            bearing = turned
    mc.Motors.turnDegrees(-scan_limit)
    return bearing

rc = mc.RoverController()

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


def main():
    
    # scan left, scan right, 
    smallest_dir = scan_for_cheese(LEFT)
    temp = scan_for_cheese(RIGHT)

    if (temp < smallest_dir):
        smallest_dir = temp

    mc.Motors.turnDegrees(smallest_dir - 20)
    extend_arm()
    pregrip_pos()
    mc.Motors.turnDegrees(25)
    grip()
    stow_arm()


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

