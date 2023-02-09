import pygame
import constants as const
from grid import Grid
import time
from pygame.math import Vector2
from math import degrees

# NORTH, EAST, SOUTH, WEST
# TURN RIGHT = +1, TURN LEFT = -1
direction_angle = [0, 270, 180, 90]
DISPLACEMENT = 1


class Robot:
    def __init__(self, screen: pygame.Surface, grid: Grid, angle: int):
        self.screen = screen
        self.grid = grid
        self.width = const.ROBOT_WIDTH
        self.height = const.ROBOT_HEIGHT
        self.location = const.ROBOT_STARTLOC
        self.angle = angle
        
        # For Animation
        self.velocity = Vector2(0.0, 0.0)
        self.speed = 10
        self.moving = False
        self.final_pixel_location = Vector2(0.0, 0.0)
        self.final_angle = 0
        self.angular_velocity = 0


        self.img = pygame.image.load(const.ROBOT_IMG_PATH)
        self.img = pygame.transform.scale(self.img, (const.ROBOT_WIDTH, const.ROBOT_HEIGHT))

        self.pixel_location = Vector2(self.get_drawing_location()[0], self.get_drawing_location()[1])
        self.rect = pygame.Rect(self.pixel_location[0], self.pixel_location[1], self.width, self.height)

    def draw_robot(self):
        if self.moving: # Check if car is moving
            if self.movement_complete(self.final_pixel_location): # Reset everything if movement completed
                self.update_robot()
                print(self.location)
            else: # Continue moving
                self.pixel_location += self.velocity.rotate(-self.angle) * 0.2
                self.angle += degrees(self.angular_velocity) * 0.2

        self.rect.topleft = self.pixel_location
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
        if self.moving:
            return
        direction = self.get_direction()
        pixel_displacement = DISPLACEMENT * const.MARGIN_BLOCK_SIZE
        if (direction == "NORTH"):
            self.location[1] += DISPLACEMENT
            final_pixel_pos = Vector2(self.pixel_location[0], self.pixel_location[1] - pixel_displacement)
        elif (direction == "EAST"):
            final_pixel_pos = Vector2(self.pixel_location[0] + pixel_displacement, self.pixel_location[1])
            self.location[0] += DISPLACEMENT
        elif (direction == "SOUTH"):
            self.location[1] -= DISPLACEMENT
            final_pixel_pos = Vector2(self.pixel_location[0], self.pixel_location[1] + pixel_displacement)
        else:
            self.location[0] -= DISPLACEMENT
            final_pixel_pos = Vector2(self.pixel_location[0] - pixel_displacement, self.pixel_location[1])

        # Set all the variables for animation    
        self.final_angle = self.angle
        self.final_pixel_location = final_pixel_pos
        self.set_velocity(0, -self.speed)
        self.moving = True

    def move_backward(self):
        if self.moving:
            return
        direction = self.get_direction()
        pixel_displacement = DISPLACEMENT * const.MARGIN_BLOCK_SIZE
        if (direction == "NORTH"):
            self.location[1] -= DISPLACEMENT
            final_pixel_pos = Vector2(self.pixel_location[0], self.pixel_location[1] + pixel_displacement)
        elif (direction == "EAST"):
            self.location[0] -= DISPLACEMENT
            final_pixel_pos = Vector2(self.pixel_location[0] - pixel_displacement, self.pixel_location[1])
        elif (direction == "SOUTH"):
            self.location[1] += DISPLACEMENT
            final_pixel_pos = Vector2(self.pixel_location[0], self.pixel_location[1] - pixel_displacement)
        else:
            self.location[0] += DISPLACEMENT
            final_pixel_pos = Vector2(self.pixel_location[0] + pixel_displacement, self.pixel_location[1])

        # Set all the variables for animation    
        self.final_angle = self.angle
        self.final_pixel_location = final_pixel_pos
        self.set_velocity(0, self.speed)
        self.moving = True

    def move_forward_left(self):
        if self.moving:
            return
        direction = self.get_direction()
        pixel_displacement = 3 * const.MARGIN_BLOCK_SIZE
        if (direction == "NORTH"):
            final_pixel_pos = Vector2(self.pixel_location[0] - pixel_displacement, self.pixel_location[1] - pixel_displacement)
            self.location = [self.location[0] - 3, self.location[1] + 3]
        elif (direction == "EAST"):
            final_pixel_pos = Vector2(self.pixel_location[0] + pixel_displacement, self.pixel_location[1] - pixel_displacement)
            self.location = [self.location[0] + 3, self.location[1] + 3]
        elif (direction == "SOUTH"):
            final_pixel_pos = Vector2(self.pixel_location[0] + pixel_displacement, self.pixel_location[1] + pixel_displacement)
            self.location = [self.location[0] + 3, self.location[1] - 3]
        else:
            final_pixel_pos = Vector2(self.pixel_location[0] - pixel_displacement, self.pixel_location[1] + pixel_displacement)
            self.location = [self.location[0] - 3, self.location[1] - 3]

        self.final_angle = self.rotate_left()
        self.final_pixel_location = final_pixel_pos
        self.set_velocity(0, -self.speed)
        self.moving = True
        self.angular_velocity = self.velocity.y / pixel_displacement * -1 # pixel_displacement is turning radius, -1 turns it in the opposite direction
        return
    
    def move_forward_right(self):
        if self.moving:
            return
        direction = self.get_direction()
        pixel_displacement = 3 * const.MARGIN_BLOCK_SIZE
        if (direction == "NORTH"):
            final_pixel_pos = Vector2(self.pixel_location[0] + pixel_displacement, self.pixel_location[1] - pixel_displacement)
            self.location = [self.location[0] + 3, self.location[1] + 3]
        elif (direction == "EAST"):
            final_pixel_pos = Vector2(self.pixel_location[0] + pixel_displacement, self.pixel_location[1] + pixel_displacement)
            self.location = [self.location[0] + 3, self.location[1] - 3]
        elif (direction == "SOUTH"):
            final_pixel_pos = Vector2(self.pixel_location[0] - pixel_displacement, self.pixel_location[1] + pixel_displacement)
            self.location = [self.location[0] - 3, self.location[1] - 3]
        else:
            final_pixel_pos = Vector2(self.pixel_location[0] - pixel_displacement, self.pixel_location[1] - pixel_displacement)
            self.location = [self.location[0] - 3, self.location[1] + 3]

        self.final_angle = self.rotate_right()
        self.final_pixel_location = final_pixel_pos
        self.set_velocity(0, -self.speed)
        self.moving = True
        self.angular_velocity = self.velocity.y / pixel_displacement #pixel_displacement is turning radius
        return
    
    def move_backward_left(self):
        if self.moving:
            return
        direction = self.get_direction()
        pixel_displacement = 3 * const.MARGIN_BLOCK_SIZE
        if (direction == "NORTH"):
            final_pixel_pos = Vector2(self.pixel_location[0] - pixel_displacement, self.pixel_location[1] + pixel_displacement)
            self.location = [self.location[0] - 3, self.location[1] - 3]
        elif (direction == "EAST"):
            final_pixel_pos = Vector2(self.pixel_location[0] - pixel_displacement, self.pixel_location[1] - pixel_displacement)
            self.location = [self.location[0] - 3, self.location[1] + 3]
        elif (direction == "SOUTH"):
            final_pixel_pos = Vector2(self.pixel_location[0] + pixel_displacement, self.pixel_location[1] - pixel_displacement)
            self.location = [self.location[0] + 3, self.location[1] + 3]
        else:
            final_pixel_pos = Vector2(self.pixel_location[0] + pixel_displacement, self.pixel_location[1] + pixel_displacement)
            self.location = [self.location[0] + 3, self.location[1] - 3]

        self.final_angle = self.rotate_right()
        self.final_pixel_location = final_pixel_pos
        self.set_velocity(0, self.speed)
        self.moving = True
        self.angular_velocity = self.velocity.y / pixel_displacement * -1 # pixel_displacement is turning radius, -1 turns it in the opposite direction
        return
    
    def move_backward_right(self):
        if self.moving:
            return
        direction = self.get_direction()
        pixel_displacement = 3 * const.MARGIN_BLOCK_SIZE
        if (direction == "NORTH"):
            final_pixel_pos = Vector2(self.pixel_location[0] + pixel_displacement, self.pixel_location[1] + pixel_displacement)
            self.location = [self.location[0] + 3, self.location[1] - 3]
        elif (direction == "EAST"):
            final_pixel_pos = Vector2(self.pixel_location[0] - pixel_displacement, self.pixel_location[1] + pixel_displacement)
            self.location = [self.location[0] - 3, self.location[1] - 3]
        elif (direction == "SOUTH"):
            final_pixel_pos = Vector2(self.pixel_location[0] - pixel_displacement, self.pixel_location[1] - pixel_displacement)
            self.location = [self.location[0] - 3, self.location[1] + 3]
        else:
            final_pixel_pos = Vector2(self.pixel_location[0] + pixel_displacement, self.pixel_location[1] - pixel_displacement)
            self.location = [self.location[0] + 3, self.location[1] + 3]

        self.final_angle = self.rotate_left()
        self.final_pixel_location = final_pixel_pos
        self.set_velocity(0, self.speed)
        self.moving = True
        self.angular_velocity = self.velocity.y / pixel_displacement # pixel_displacement is turning radius, -1 turns it in the opposite direction
        return

    def rotate_left(self):
        index = direction_angle.index(self.angle) # Current direction by index
        # Rotating left means decrementing the index in the list
        if (index == 0):
            return direction_angle[3]
        else:
            return direction_angle[index-1]
    
    def rotate_right(self):
        index = direction_angle.index(self.angle) # Current direction by index
        # Rotating right means incrementing the index in the list
        if (index == 3):
            return direction_angle[0]
        else:
            return direction_angle[index+1]
    
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
        
    def set_velocity(self, a, b):
        self.velocity[0] = a
        self.velocity[1] = b

    def movement_complete(self, final_pos):
        xdiff = abs(self.pixel_location[0] - self.final_pixel_location[0])
        ydiff = abs(self.pixel_location[1] - self.final_pixel_location[1])
        if (xdiff < 4 and ydiff < 4):
            return True
        #print("movement not complete, xdiff=" + str(xdiff) + ", ydiff=" + str(ydiff)) FOR DEBUGGING HOLY SHIT I WANT TO KMS DOING THIS
        return False
    
    def update_robot(self):
        self.angle = self.final_angle
        self.pixel_location = self.final_pixel_location
        self.moving = False
        self.velocity = Vector2(0.0, 0.0)
        self.angular_velocity = 0