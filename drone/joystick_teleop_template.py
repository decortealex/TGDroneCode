import pygame


pygame.init()
size = [100, 100]
screen = pygame.display.set_mode(size)
pygame.display.set_caption("Drone Teleop")

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

if pygame.joystick.get_count() == 0:
    print("No joysticks found")
    done = True
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Initialized %s" % (joystick.get_name()))
    print("Number of buttons %d. Number of axis %d, Number of hats %d" %
          (joystick.get_numbuttons(), joystick.get_numaxes(),
           joystick.get_numhats()))

left_trigger_initialized = False
right_trigger_initialized = False

# -------- Main Program Loop -----------

while not done:
    # EVENT PROCESSING STEP
    for event in pygame.event.get():  # User did something
        if event.type == pygame.QUIT:  # If user clicked close
            done = True  # Flag that we are done so we exit this loop

    if joystick.get_button(0) == 1:
        print("Ow")

    if joystick.get_button(1) == 1:
        print(":(")

    if joystick.get_button(2) == 1:
        print("Quit it!")

    if joystick.get_button(3) == 1:
        print("lOL")

    if joystick.get_button(4) == 1:
        print("Button 4 pressed")

    if joystick.get_button(5) == 1:
        print("Button 5 pressed")

    if joystick.get_button(6) == 1:
        print("Button 6 pressed")

    if joystick.get_button(7) == 1:
        print("Button 7 pressed")

    if abs(joystick.get_axis(0)) > 0.05 or abs(joystick.get_axis(1)) > 0.05:
        print("Left stick %f, %f" % (joystick.get_axis(0),
                                     joystick.get_axis(1)))

    if abs(joystick.get_axis(3)) > 0.05 or abs(joystick.get_axis(4)) > 0.05:
        print("Right stick %f, %f" % (joystick.get_axis(3),
                                      joystick.get_axis(4)))

    # Triggers initialize at 0 and then reset to -1
    if joystick.get_axis(2) != 0:
        left_trigger_initialized = True

    if joystick.get_axis(5) != 0:
        right_trigger_initialized = True

    if left_trigger_initialized and abs(joystick.get_axis(2) > -.95):
        print("Left trigger %f" % joystick.get_axis(2))

    if right_trigger_initialized and abs(joystick.get_axis(5) > -.95):
        print("Right trigger %f" % joystick.get_axis(5))

    (hat_x, hat_y) = joystick.get_hat(0)
    if (hat_x != 0 or hat_y !=0):
        print("Hat %d, %d" % (hat_x, hat_y))

    # Limit to 20 frames per second
    clock.tick(20)

# Close the window and quit.
pygame.quit()
