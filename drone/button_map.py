import pygame
import sys

pygame.init()
pygame.display.set_mode((100, 100))
pygame.key.set_repeat(50)

pygame.joystick.init()

my_joystick = pygame.joystick.Joystick(0)
my_joystick.init()

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()

        if event.type == pygame.JOYBUTTONDOWN:
            if event.button == 0:
                print("Button 'X' was pressed.")
            if event.button == 1:
                print("Button 'A' was pressed.")
            if event.button == 2:
                print("Button 'B' was pressed.")
            if event.button == 3:
                print("Button 'Y' was pressed.")
            if event.button == 4:
                print("Button 'LB' was pressed.")
            if event.button == 5:
                print("Button 'RB' was pressed.")
            if event.button == 6:
                print("Button 'LT' was pressed.")
            if event.button == 7:
                print("Button 'RT' was pressed.")
            if event.button == 8:
                print("Button 'Back' was pressed.")
            if event.button == 9:
                print("Button 'Start' was pressed.")


        if event.type == pygame.JOYAXISMOTION:
            if event.axis == 0:
                print("X axis of Left Joystick is at: " + str(my_joystick.get_axis(0)))
            if event.axis == 1:
                print("Y axis of Left Joystick is at: " + str(my_joystick.get_axis(1)))
            if event.axis == 2:
                print("X axis of Right Joystick is at: " + str(my_joystick.get_axis(2)))
            if event.axis == 3:
                print("Y axis of Right Joystick is at: " + str(my_joystick.get_axis(3)))

        if event.type == pygame.JOYHATMOTION:
            if my_joystick.get_hat(0) == (0, 1):
                print("DPad is Up")
            elif my_joystick.get_hat(0) == (0, -1):
                print("DPad is Down")
            elif my_joystick.get_hat(0) == (-1, 0):
                print("DPad is Left")
            elif my_joystick.get_hat(0) == (1, 0):
                print("DPad is Right")
