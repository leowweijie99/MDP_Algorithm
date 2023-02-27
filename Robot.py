import pygame
import constants as const
from grid import Grid
import time
from pygame.math import Vector2
from math import degrees
from enum import IntEnum
from cell import CellStatus

# NORTH, EAST, SOUTH, WEST
# TURN RIGHT = +1, TURN LEFT = -1
direction_angle = [0, -90, 180, 90]

class RobotMoves(IntEnum):
    FORWARD = 0
    BACKWARD = 1
    FORWARD_LEFT = 2
    FORWARD_RIGHT = 3
    BACKWARD_LEFT = 4
    BACKWARD_RIGHT = 5
    SCAN = 6

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
        self.turn = False # Use to determine when to turn (Forward = turn first, Backward = turn second)
        self.dest_queue = []
        self.movement_queue = []

        self.img = pygame.image.load(const.ROBOT_IMG_PATH)
        self.img = pygame.transform.scale(self.img, (const.ROBOT_WIDTH, const.ROBOT_HEIGHT))

        self.pixel_location = Vector2(self.get_drawing_location()[0], self.get_drawing_location()[1])
        self.rect = pygame.Rect(self.pixel_location[0], self.pixel_location[1], self.width, self.height)

    def is_moving(self):
        return self.moving

    def draw_robot(self):
        if self.moving: # Check if car is moving
            self.animate_robot()
        else: # If car is not moving
            if (len(self.movement_queue) > 0):
                self.execute_movement(self.movement_queue[0])
                self.movement_queue.pop(0)
        # Draw robot
        self.rect.topleft = self.pixel_location
        rotated_image = pygame.transform.rotate(self.img, self.angle)
        new_rect = rotated_image.get_rect(center = self.rect.center)
        pygame.draw.rect(self.screen, const.GREY, self.rect)
        self.screen.blit(rotated_image, new_rect)
        return
    
    def animate_robot(self):
        if self.movement_complete(self.dest_queue[0]): # If movement is completed
            self.update_robot()
            if len(self.dest_queue) != 0: # If robot is only halfway
                self.turn = not self.turn # Invert turn. if was turning, time to move straight
            else:                         # MOVEMENT COMPLETED
                #print(self.location)
                self.grid.get_cell(int(self.location[0]), int(self.location[1])).status = CellStatus.PATH
                self.turn = False
        else:                             # Continue moving
            if (self.turn):               # Need to turn
                self.angle += degrees(self.angular_velocity) * 0.2
            self.pixel_location += self.velocity.rotate(-self.angle) * 0.2
                     

    def get_drawing_location(self):
        loc = self.location
        pixel_loc = self.grid.get_pixel_measure(loc[0], 19 - loc[1]) # This gives the bottom left position instead of top left (to draw)
        drawing_loc = [pixel_loc[0] - const.BLOCK_SIZE, pixel_loc[1] - const.BLOCK_SIZE] # Transform to top left
        return drawing_loc

    def move_forward(self, distance: int):
        if self.moving:
            return
        distance //= 10 # original is in cm, move_foward(10) should move 1 cell
        direction = self.get_direction()

        pixel_vector = self.transform_vector(Vector2(0, -distance), -self.angle) * const.MARGIN_BLOCK_SIZE
        grid_vector = self.transform_vector(Vector2(0, distance), self.angle)
                                            
        self.location += grid_vector
        self.dest_queue.append(self.pixel_location + pixel_vector)

        # Set all the variables for animation    
        self.final_angle = self.angle
        self.set_velocity(0, -self.speed)
        self.moving = True

    def move_backward(self, distance: int):
        if self.moving:
            return
        distance //= 10 # original is in cm, move_foward(10) should move 1 cell
        direction = self.get_direction()

        pixel_vector = self.transform_vector(Vector2(0, distance), -self.angle) * const.MARGIN_BLOCK_SIZE
        grid_vector = self.transform_vector(Vector2(0, -distance), self.angle)
                                            
        self.location += grid_vector
        self.dest_queue.append(self.pixel_location + pixel_vector)

        # Set all the variables for animation    
        self.final_angle = self.angle
        self.set_velocity(0, self.speed)
        self.moving = True

    def move_forward_left(self):
        if self.moving:
            return
        direction = self.get_direction()

        # DEFAULT IS NORTH
        pixel_vector = self.transform_vector(Vector2(-3, -1), -self.angle) * const.MARGIN_BLOCK_SIZE
        grid_vector = self.transform_vector(Vector2(-3, 1), self.angle)
        turn_vector = self.transform_vector(Vector2(-1, -1), -self.angle) * const.MARGIN_BLOCK_SIZE
        
        self.location += grid_vector
        self.dest_queue.append(self.pixel_location + turn_vector) # TurnPos goes in first
        self.dest_queue.append(self.pixel_location + pixel_vector) # FinalPos last
        self.turn = True # Tells animator to turn first

        self.final_angle = self.rotate_left()
        self.set_velocity(0, -self.speed)
        self.moving = True
        self.angular_velocity = self.velocity.y / const.MARGIN_BLOCK_SIZE * -1 # Turning Radius, -1 turns it in the opposite direction
        return
    
    def move_forward_right(self):
        if self.moving:
            return
        direction = self.get_direction()

        pixel_vector = self.transform_vector(Vector2(3, -1), -self.angle) * const.MARGIN_BLOCK_SIZE
        grid_vector = self.transform_vector(Vector2(3, 1), self.angle)
        turn_vector = self.transform_vector(Vector2(1, -1), -self.angle) * const.MARGIN_BLOCK_SIZE

        self.location += grid_vector
        self.dest_queue.append(self.pixel_location + turn_vector) # TurnPos goes in first
        self.dest_queue.append(self.pixel_location + pixel_vector) # FinalPos last
        self.turn = True # Tells animator to turn first
        self.final_angle = self.rotate_right()
        self.set_velocity(0, -self.speed)
        self.moving = True
        self.angular_velocity = self.velocity.y / const.MARGIN_BLOCK_SIZE # Turning Radius
        return
    
    def move_backward_left(self):
        if self.moving:
            return
        direction = self.get_direction()

        # DEFAULT IS NORTH
        pixel_vector = self.transform_vector(Vector2(-1, 3), -self.angle) * const.MARGIN_BLOCK_SIZE
        grid_vector = self.transform_vector(Vector2(-1, -3), self.angle)
        turn_vector = self.transform_vector(Vector2(1, -1), -self.angle) * const.MARGIN_BLOCK_SIZE
        
        self.location += grid_vector
        final_pixel_location = self.pixel_location + pixel_vector
        self.dest_queue.append(final_pixel_location + turn_vector) # TurnPos goes in first
        self.dest_queue.append(final_pixel_location) # FinalPos last
        self.turn = False # Tells animator to turn last

        self.final_angle = self.rotate_right()
        self.set_velocity(0, self.speed)
        self.moving = True
        self.angular_velocity = self.velocity.y / const.MARGIN_BLOCK_SIZE * -1 # Turning Radius, -1 turns it in the opposite direction
        return
    
    def move_backward_right(self):
        if self.moving:
            return
        direction = self.get_direction()
        n = const.MARGIN_BLOCK_SIZE

        pixel_vector = self.transform_vector(Vector2(1, 3), -self.angle) * const.MARGIN_BLOCK_SIZE
        grid_vector = self.transform_vector(Vector2(1, -3), self.angle)
        turn_vector = self.transform_vector(Vector2(-1, -1), -self.angle) * const.MARGIN_BLOCK_SIZE

        self.location += grid_vector
        final_pixel_location = self.pixel_location + pixel_vector
        self.dest_queue.append(final_pixel_location + turn_vector) # TurnPos goes in first
        self.dest_queue.append(final_pixel_location) # FinalPos last
        self.turn = False # Tells animator to turn last

        self.final_angle = self.rotate_left()
        self.set_velocity(0, self.speed)
        self.moving = True
        self.angular_velocity = self.velocity.y / const.MARGIN_BLOCK_SIZE # Turning Radius
        return

    # Rotating left means decrementing the index in the list
    def rotate_left(self):
        index = direction_angle.index(self.angle) # Current direction by index
        return direction_angle[(index - 1) % 4]
    
    # Rotating right means incrementing the index in the list
    def rotate_right(self):
        index = direction_angle.index(self.angle) # Current direction by index
        return direction_angle[(index + 1) % 4]
    
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
        xdiff = abs(self.pixel_location[0] - final_pos[0])
        ydiff = abs(self.pixel_location[1] - final_pos[1])
        if (xdiff < 3 and ydiff < 3):
            return True
        return False
    
    def update_robot(self):
        if (self.turn):
            self.angle = self.final_angle
        self.pixel_location = self.dest_queue[0]
        self.dest_queue.pop(0)
        if (len(self.dest_queue) == 0):
            self.moving = False
            self.velocity = Vector2(0.0, 0.0)
            self.angular_velocity = 0

    def execute_movement(self, movement):
        #print("executing " + str(movement))
        if movement == RobotMoves.FORWARD:
            self.move_forward(10)
        elif movement == RobotMoves.BACKWARD:
            self.move_backward(10)
        elif movement == RobotMoves.BACKWARD_LEFT:
            self.move_backward_left()
        elif movement == RobotMoves.BACKWARD_RIGHT:
            self.move_backward_right()
        elif movement == RobotMoves.FORWARD_LEFT:
            self.move_forward_left()
        elif movement == RobotMoves.FORWARD_RIGHT:
            self.move_forward_right()   
        elif type(movement) is list:
            self.scan(movement)       

    def print_queue(self):
        print(self.movement_queue[0])
        self.movement_queue.pop(0)

    def transform_vector(self, vector: Vector2, angle: int):
        return vector.rotate(angle)

    def scan(self, position: list):
        time.sleep(1)
        robot_xpos = position[0]
        robot_ypos = position[1]

        self.grid.cells[robot_xpos][robot_ypos].status = CellStatus.VISITED_GOAL