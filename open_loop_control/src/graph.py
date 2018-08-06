import matplotlib.pyplot as plt
import numpy as np

### FUNCTIONS ###

def lincoln_path(t):
    """ Does a lincoln labs path. """
    return ((4.0*np.pi/400.0) * np.cos(8.0 * np.pi * t / 25.0),
            (9.0*np.pi/800.0) * np.cos(6.0 * np.pi * t / 25.0),
            0.0)

# Moustache
def moustache(t):
    """ Its a moustache! """
    return ((2.0*np.pi/25.0) * np.sin(2.0*np.pi*t/25.0),
            (2.0*np.pi/125.0)*((4.0*np.sin(8.0*np.pi*t/25.0)) - np.cos(2.0*np.pi*t/25.0)),
            0.0)

### GATHER THE POINTS ###
def gather_points(func, duration):
    yield (0, 0) # Beginning.

    current_location = [0, 0]
    current_time = 0.0
    while current_time <= duration:
        current_time += 0.1
        vx, vy, vz = func(current_time)
        current_location[0] += vx
        current_location[1] += vy
        print("{:.2f}: ({:.2f}, {:.2f}) with v: ({:.2f}, {:.2f})".format(current_time, *current_location, vx, vy))
        yield tuple(current_location)


### DISPLAY THE PLOT, LIVE ###
for x, y in gather_points(lincoln_path, 25):
    plt.pause(0.1)
    plt.plot(x, y, 'bo')

plt.show()