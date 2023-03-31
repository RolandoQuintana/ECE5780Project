import inputs;
import time
import math

#
#  1      2
#
#
#
#  3      4
#

ZERO_TOLERANCE = 0.1
STATIC_WHEEL_MULTIPLIER = 0.9
DYNAMIC_WHEEL_MULTIPLIER = 1.3
TOTALSTEPS = 2**16 / 2
x = 0
y = 0
wheelDoodies = [0,0,0,0]

try:
    inputs.get_gamepad()
except Exception as e:
    print(e)  
    exit(0)  

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
            # this will be the duty cycle for the set of wheels that change direction in a quadrant
            m = math.sqrt(x**2 + y**2) * (math.atan(abs(y/x if x!= 0 else 0.001)) - math.pi/4)*DYNAMIC_WHEEL_MULTIPLIER
            # duty cycle for set of wheels that stay the same in a quadrant
            ma = math.sqrt(x**2 + y**2) * STATIC_WHEEL_MULTIPLIER
            # clamp both values
            m = m if m < 1 else 1
            ma = ma if ma < 1 else 1
            # right side of plane
            if x > ZERO_TOLERANCE: 
                # first quadrant
                if y > ZERO_TOLERANCE:
                    wheelDoodies[0] = m
                    wheelDoodies[1] = ma
                    wheelDoodies[2] = ma
                    wheelDoodies[3] = m
                # fourth quadrant
                elif y < -ZERO_TOLERANCE:
                    wheelDoodies[0] = -ma
                    wheelDoodies[1] = -m
                    wheelDoodies[2] = -m
                    wheelDoodies[3] = -ma
                # dead zone, only go right
                else:
                    wheelDoodies[0] = -ma
                    wheelDoodies[1] = ma
                    wheelDoodies[2] = ma
                    wheelDoodies[3] = -ma
            # left side of plane
            elif x < -ZERO_TOLERANCE:
                # second quadrant
                if y > ZERO_TOLERANCE:
                    wheelDoodies[0] = ma
                    wheelDoodies[1] = m
                    wheelDoodies[2] = m
                    wheelDoodies[3] = ma
                # third quadrant
                elif y < -ZERO_TOLERANCE:
                    wheelDoodies[0] = -m
                    wheelDoodies[1] = -ma
                    wheelDoodies[2] = -ma
                    wheelDoodies[3] = -m
                # dead zone, only go left
                else: 
                    wheelDoodies[0] = ma
                    wheelDoodies[1] = -ma
                    wheelDoodies[2] = -ma
                    wheelDoodies[3] = ma
            # horizontal deadzone
            else:
                # go up
                if y > ZERO_TOLERANCE:
                    wheelDoodies[0] = ma
                    wheelDoodies[1] = ma
                    wheelDoodies[2] = ma
                    wheelDoodies[3] = ma
                # go down
                elif y < -ZERO_TOLERANCE:
                    wheelDoodies[0] = -ma
                    wheelDoodies[1] = -ma
                    wheelDoodies[2] = -ma
                    wheelDoodies[3] = -ma
                # don't move
                else:
                    wheelDoodies[0] = 0
                    wheelDoodies[1] = 0
                    wheelDoodies[2] = 0
                    wheelDoodies[3] = 0
        print(wheelDoodies)
            
                    
    except Exception as e:
        print("Controller disconnected" + str(e))
        exit(0)