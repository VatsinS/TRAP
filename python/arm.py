import Rpi.GPIO as GPIO
from RpiMotorLib import rpiservolib


def testmove(degr):
    myservotest = rpiservolib.SG90servo("servoone")
    #motor = SG90servo.__init__("servoone")
    #dutycycle = myservotest.convert_from_degree(myservotest, degr)
    myservotest.servo_move(myservotest, 12, myservotest.convert_from_degree(myservotest, degr)) # using pin 12

def main():
    angle = 90
    testmove(angle)