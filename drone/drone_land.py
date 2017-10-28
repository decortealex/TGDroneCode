import pygame
import sys
from commands import *
from bebop import Bebop
import logging

logging.basicConfig(level=logging.DEBUG)

print("Connecting to drone...")
drone = Bebop()
# drone.trim()
print("Connected.")

drone.land()
