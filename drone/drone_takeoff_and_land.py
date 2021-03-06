import pygame
import sys
from commands import *
from bebop import Bebop
import logging

logging.basicConfig(level=logging.DEBUG)

print("Connecting to drone...")
drone = Bebop()
drone.trim()
print("Connected.")


drone.takeoff()
drone.wait(.2)
drone.update(cmd=movePCMDCmd(True, 0, 40, 0, 0))
drone.wait(2.3)
drone.update(cmd=movePCMDCmd(True, 0, 0, 0, 0))
drone.wait(.2)
drone.land()