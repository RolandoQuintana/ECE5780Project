import inputs
import serial
import math
import numpy

#    FRONT
#  1      2
#
#
#
#  3      4
#    BACK


cereal = serial.Serial('COM3', 9600)

# For tuning 
ZERO_TOLERANCE = 0.15
STATIC_WHEEL_MULTIPLIER = 1
DYNAMIC_WHEEL_MULTIPLIER = 1.3
# Don't mess with this one, is good
TOTALSTEPS = 2**16 / 2
x, y = 0, 0

# Check if theres even a controller connected
try:
    inputs.get_gamepad()
except Exception as e:
    print(e)
    exit(0)

def calculate_duties(x, y):
    # this will be the duty cycle for the set of wheels that change direction in a quadrant
    m = math.sqrt(x**2 + y**2) * (math.atan(abs(y/(x if x != 0 else 0.001))) - math.pi/4) * DYNAMIC_WHEEL_MULTIPLIER
    # duty cycle for set of wheels that stay the same in a quadrant
    ma = math.sqrt(x**2 + y**2) * STATIC_WHEEL_MULTIPLIER
    # clamp both values
    m = min(m, 1)
    ma = min(ma, 1)
    # right side of plane
    if x > ZERO_TOLERANCE:
        # first quadrant
        if y > ZERO_TOLERANCE:
            return m, ma, ma, m
        # fourth quadrant
        elif y < -ZERO_TOLERANCE:
            return -ma, -m, -m, -ma
        # dead zone, only go right
        else:
            return -ma, ma, ma, -ma
    # left side of plane
    elif x < -ZERO_TOLERANCE:
        # second quadrant
        if y > ZERO_TOLERANCE:
            return ma, m, m, ma
        # third quadrant
        elif y < -ZERO_TOLERANCE:
            return -m, -ma, -ma, -m
        # dead zone, only go left
        else:
            return ma, -ma, -ma, ma
    # horizontal deadzone
    else:
        # go up
        if y > ZERO_TOLERANCE:
            return ma, ma, ma, ma
        # go down
        elif y < -ZERO_TOLERANCE:
            return -ma, -ma, -ma, -ma
        # don't move
        else:
            return 0, 0, 0, 0
        
# Helper for turning those binary numbers back into signed 8 bit ints
def printWheelSpeeds(wheel_duties):
    return numpy.int8(wheel_duties[0]), numpy.int8(wheel_duties[1]), numpy.int8(wheel_duties[2]), numpy.int8(wheel_duties[3])
        
while True:
    # try here because events will throw an error if controller gets disconnected
    try:
        events = inputs.get_gamepad()
        for event in events:
            # only get the left joystick events
            if event.code == inputs.ABSOLUTE_AXES[0][1]:
                x = event.state / TOTALSTEPS
            if event.code == inputs.ABSOLUTE_AXES[1][1]:
                y = event.state / TOTALSTEPS
                
            # Turn the floats into signed 8 bit ints, then turn into byte array
            wheel_duties = bytes([numpy.uint8(i * 100) for i in calculate_duties(x,y)])
            
            # SEND IT
            cereal.write(wheel_duties)
            
            # For debugging purposes
            print(printWheelSpeeds(wheel_duties))

    except Exception as e:
        print("Controller disconnected" + str(e))
        exit(0)