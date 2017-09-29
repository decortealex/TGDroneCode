from bebop import Bebop
import logging

logging.basicConfig()

print("Connecting to drone...")
drone = Bebop(loggingLevel=logging.DEBUG)
print("Connected.")

drone.wait(5)