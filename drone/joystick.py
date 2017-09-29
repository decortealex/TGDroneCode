import pygame
import sys
from commands import *
from bebop import Bebop

drive_speed = 50
turn_speed = 50

print ("Connecting to drone...");
metalog=None
drone = Bebop( metalog=metalog, onlyIFrames=True )
drone.trim()

print ("Connected.");

drive_speed = 50
turn_speed = 50

pygame.init()
pygame.display.set_mode((100, 100))
pygame.key.set_repeat(50)

pygame.joystick.init()

my_joystick = pygame.joystick.Joystick(0)
my_joystick.init()

while True:
    for event in pygame.event.get():
        #leftPower = 0
        #rightPower = 0
        video = True
        if event.type == pygame.QUIT:
            # print "hi"
            sys.exit()

        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == 3:
                drone.takeoff();
                print("Drone taking off.")
            if event.button == 0:
                drone.land();
                print("Drone landing.")
            if event.button == 7:
                drone.update(cmd=movePCMDCmd(True, 0, 0, 50, 0))
                print("Drone spinning clockwise.")
            if event.button == 6:
                drone.update(cmd=movePCMDCmd(True, 0, 0, -50, 0))
                print("Drone spinning counterclockwise.")
            if event.button == 2:
                print("Drone ascending.")
                drone.update(cmd=movePCMDCmd(True, 0, 0, 0, 50))
                drone.wait(0.050)
            if event.button == 1:
                print("Drone descending.")
                drone.update(cmd=movePCMDCmd(True, 0, 0, 0, -50))
                drone.wait(0.050)
            # if event.button == 4:
            #     print("Taking picture...")
            #     drone.update(cmd=takePictureCmd())
            # if event.button == 5:
            #     if video is True:
            #         print("Enabling video stream.")
            #         drone.update(cmd=videoStreamingCmd(enable=True), ackRequest=True)
            #         video = False
            #     else:
            #         print("Disabling video stream.")
            #         drone.update(cmd=videoStreamingCmd(enable=False), ackRequest=True)
            #         video = Truebutton_map.py

        if event.type == pygame.JOYAXISMOTION:
            drone.update(cmd=movePCMDCmd(True, (50 * my_joystick.get_axis(2)), (-50 * my_joystick.get_axis(1)), 0, 0))
            print("Forwards/Backwards power is " + str((-50 * my_joystick.get_axis(1))))
            print("Left/Right power is " + str((50 * my_joystick.get_axis(2))))
            drone.wait(0.100)
            # if event.axis == 1:
            #     if event.value < -0.05:
            #         print("Forward power is " + str((-50 * my_joystick.get_axis(1))))
            #         drone.update(cmd=movePCMDCmd(True, 0, -50 * my_joystick.get_axis(1), 0, 0))
            #     elif event.value > 0.05:
            #         print("Backward power is " + (-50 * str(my_joystick.get_axis(1))))
            #         drone.update(cmd=movePCMDCmd(True, 0, -50 * my_joystick.get_axis(1), 0, 0))
            # if event.axis == 2:
            #     if event.value > 0.05:
            #         print "Right power is " + str((50 * my_joystick.get_axis(2)))
            #         drone.update(cmd=movePCMDCmd(True, 50 * my_joystick.get_axis(2), 0, 0, 0))
            #         drone.wait(0.100)
            #     if event.value < -0.05:
            #         print "Left power is " + str((50 * my_joystick.get_axis(2)))
            #         drone.update(cmd=movePCMDCmd(True, 50 * my_joystick.get_axis(2), 0, 0, 0))
            #         drone.wait(0.100)

        # if event.type == pygame.JOYBUTTONUP:
        #     print("Joystick button released.")
    if drone.flyingState == 2:
        print("Drone Hover")
