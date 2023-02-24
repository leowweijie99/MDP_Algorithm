from enum import IntEnum
import constants as const
import pygame

class FacingDirection(IntEnum):
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3

class Goal:
    def __init__(self, x, y, facing_direction: int, id = -1):
        self.x = x
        self.y = y
        self.id = id
        self.facing_direction = facing_direction
        self.data = (self.x, self.y, self.facing_direction, self.id)

    def draw_goal(self, cell_surface, x, y):
        if self.facing_direction == FacingDirection.UP:
            print("Colour cell red at ", x, y+3)
        elif self.facing_direction == FacingDirection.RIGHT:
            print("Colour cell red at ", x+3, y)
        elif self.facing_direction == FacingDirection.DOWN:
            print("Colour cell red at ", x, y-3)
        elif self.facing_direction == FacingDirection.LEFT:
            print("Colour cell red at ", x-3, y)

    