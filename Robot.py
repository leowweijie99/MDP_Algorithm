import pygame
import constants as const
from grid import Grid
import time

# NORTH, EAST, SOUTH, WEST
# TURN RIGHT = +1, TURN LEFT = -1
direction_angle = [0, 270, 180, 90]
VELOCITY = 1 # How many meters in 1 movement

class Robot:
    def __init__(self, screen: pygame.Surface, grid: Grid, angle: int):
        self.screen = screen
        self.grid = grid
        self.width = const.ROBOT_WIDTH
        self.height = const.ROBOT_HEIGHT
        self.location = const.ROBOT_STARTLOC
        self.angle = angle

        self.img = pygame.image.load(const.ROBOT_IMG_PATH)
        self.img = pygame.transform.scale(self.img, (const.ROBOT_WIDTH, const.ROBOT_HEIGHT))

        pixel_location = self.get_drawing_location()
        self.rect = pygame.Rect(pixel_location[0], pixel_location[1], self.width, self.height)

    def draw_robot(self):
        self.rect.topleft = self.get_drawing_location()
        rotated_image = pygame.transform.rotate(self.img, self.angle)
        new_rect = rotated_image.get_rect(center = self.rect.center)
        pygame.draw.rect(self.screen, const.GREY, self.rect)
        self.screen.blit(rotated_image, new_rect)
        return
    
    def get_drawing_location(self):
        loc = self.location
        pixel_loc = self.grid.get_pixel_measure(loc[0], 19 - loc[1]) # This gives the bottom left position instead of top left (to draw)
        drawing_loc = [pixel_loc[0] + const.MARGIN, pixel_loc[1] - (2 * const.MARGIN_BLOCK_SIZE) + const.MARGIN] # Transform to top left
        return drawing_loc

    def move_forward(self):
        direction = self.get_direction()
        if (direction == "NORTH"):
            self.location[1] += VELOCITY
        elif (direction == "EAST"):
            self.location[0] += VELOCITY
        elif (direction == "SOUTH"):
            self.location[1] -= VELOCITY
        else:
            self.location[0] -= VELOCITY

    def move_backward(self):
        direction = self.get_direction()
        if (direction == "NORTH"):
            self.location[1] -= VELOCITY
        elif (direction == "EAST"):
            self.location[0] -= VELOCITY
        elif (direction == "SOUTH"):
            self.location[1] += VELOCITY
        else:
            self.location[0] += VELOCITY

    def move_forward_left(self):
        self.move_forward()
        self.rotate_left()
        self.move_forward()
        return
    
    def move_forward_right(self):
        self.move_forward()
        self.rotate_right()
        self.move_forward()
        return
    
    def move_backward_left(self):
        self.move_backward()
        self.rotate_right()
        self.move_backward()
        return
    
    def move_backward_right(self):
        self.move_backward()
        self.rotate_left()
        self.move_backward()
        return

    def rotate_left(self):
        index = direction_angle.index(self.angle) # Current direction by index
        # Rotating left means decrementing the index in the list
        if (index == 0):
            index = 3
        else:
            index -= 1
        self.angle = direction_angle[index]
        return
    
    def rotate_right(self):
        index = direction_angle.index(self.angle) # Current direction by index
        # Rotating right means incrementing the index in the list
        if (index == 3):
            index = 0
        else:
            index += 1
        self.angle = direction_angle[index]
        return
    
    def get_direction(self):
        index = direction_angle.index(self.angle) # Current direction by index
        if (index == 0):
            return "NORTH"
        elif (index == 1):
            return "EAST"
        elif (index == 2):
            return "SOUTH"
        else:
            return "WEST"
