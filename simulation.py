import pygame
import time

from controller import PID_Controller
from fluid import Fluid
from vehicle import Vehicle


MAX_DEPTH = 15

RGB_BLUE = (41, 128, 185)
RGB_GREEN = (46, 204, 113)
RGB_RED = (231, 75, 60)
RGB_ORANGE = (230, 126, 34)
RGB_BLACK = (0, 0, 0)
RGB_WHITE = (255, 255, 255)

DISPLAY_OFFSET = 40
DISPLAY_SCALE = 30
DISPLAY_SIZE = (250, 560)
TOLERANCE = 0.2

TIME_INTERVAL = 0.05

PID_VARIABLES = 3, 0, 70, 1.8


def main():
    pygame.init()
    screen = pygame.display.set_mode(DISPLAY_SIZE)
    surface = pygame.display.get_surface()
    pygame.display.set_caption("Vehicle Simulation")
    clock = pygame.time.Clock()

    fluid = Fluid(MAX_DEPTH, fluid_density=50.1, dt=TIME_INTERVAL)
    controller = PID_Controller(*PID_VARIABLES)
    vehicle = Vehicle(controller, tolerance=TOLERANCE)

    count_within_threshold = 0
    sp_color = RGB_RED
    start_time = time.time()

    while True:
        fluid.update_vehicle(vehicle)
        if not vehicle.reached_goal():
            count_within_threshold = 0
            sp_color = RGB_RED
        else:
            count_within_threshold += 1
            sp_color = RGB_GREEN
        if count_within_threshold == 40:
            print(f"Reached goal in {time.time()-start_time:.3f} seconds.")

        screen.fill(RGB_BLACK)
        w = surface.get_width()
        pygame.draw.rect(
                screen, 
                RGB_BLUE, 
                (0, 
                DISPLAY_OFFSET, 
                w, 
                (MAX_DEPTH + vehicle.get_height()) * DISPLAY_SCALE), 
                0)  # water
        pygame.draw.rect(
                screen, 
                RGB_WHITE,
                (0, 0, w, DISPLAY_OFFSET),
                0)  # water surface
        pygame.draw.rect(
                screen, 
                RGB_ORANGE, 
                (w/2, 
                vehicle.get_depth() * DISPLAY_SCALE + DISPLAY_OFFSET,
                vehicle.get_width() * DISPLAY_SCALE, 
                vehicle.get_height() * DISPLAY_SCALE), 
                0)  # vehicle
        pygame.draw.lines(
                screen, 
                sp_color, 
                False,
                ((0, vehicle.get_setpoint() * DISPLAY_SCALE + DISPLAY_OFFSET), 
                (w, vehicle.get_setpoint() * DISPLAY_SCALE + DISPLAY_OFFSET)), 
                3)  # line
        pygame.display.update()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
            if event.type == pygame.MOUSEBUTTONUP:
                pos = pygame.mouse.get_pos()
                new_sp = min(MAX_DEPTH, max(0, (pos[1] - DISPLAY_OFFSET) / DISPLAY_SCALE))
                vehicle.set_setpoint(new_sp)
                print(f"New goal: {new_sp:.3f}m")
                start_time = time.time()

        clock.tick(1/TIME_INTERVAL)

if __name__ == "__main__":
    main()