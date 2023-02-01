from enum import IntEnum
import pygame

class FacingDirection(IntEnum):
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3

class Obstacle:
    def __init__(self, x, y, facing_direction: FacingDirection = FacingDirection.UP):
        self.x = x
        self.y = y
        self.facing_direction = facing_direction
        self.visited = False
        self.host_square = str(x) + '-' + str(y)
        self.target = ''

    def has_been_visited(self):
        if self.visited == True:
            return True
        else:
            return False

    def on_click(self):

        facing_directions = [FacingDirection.UP, FacingDirection.RIGHT, FacingDirection.DOWN, FacingDirection.LEFT]

        current_facing_direction = facing_directions.index(self.facing_direction)

        self.facing_direction = facing_directions[(current_facing_direction + 1)%4]
        
