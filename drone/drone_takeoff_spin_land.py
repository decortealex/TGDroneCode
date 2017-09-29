import pygame
import sys
from commands import *
from bebop import Bebop


print("Connecting to drone...")
drone = Bebop()
drone.trim()
print("Connected.")


drone.takeoff()
drone.wait(5)
drone.update(cmd=movePCMDCmd(True, 0, 0, -50, 0))
drone.wait(3)
drone.land()
