from enum import IntEnum
import constants as const
import pygame

margin = const.MARGIN
length = const.BLOCK_SIZE


class FacingDirection(IntEnum):
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3

class Obstacle:
    def __init__(self, x, y, id, facing_direction: FacingDirection = FacingDirection.UP):
        self.x = x
        self.y = y
        self.id = id
        self.facing_direction = facing_direction
        self.visited = False
        self.host_square = str(x) + '-' + str(y)
        self.target = ''
        self.goal_cell = None # This goal in obstacle is pointing towards the cell

    def has_been_visited(self):
        if self.visited == True:
            return True
        else:
            return False

    def draw_obstacle(self, cell_surface, x, y):
        if self.facing_direction == FacingDirection.UP:
            pygame.draw.line(cell_surface, const.NEON_PINK, (x, y), (x + length, y), 8)
        elif self.facing_direction == FacingDirection.RIGHT:
            pygame.draw.line(cell_surface, const.NEON_PINK, (x + length-const.MARGIN, y), (x + length-const.MARGIN, y + length), 8)
        elif self.facing_direction == FacingDirection.DOWN:
            pygame.draw.line(cell_surface, const.NEON_PINK, (x, y + length-const.MARGIN), (x + length, y + length-const.MARGIN), 8)
        elif self.facing_direction == FacingDirection.LEFT:
            pygame.draw.line(cell_surface, const.NEON_PINK, (x, y), (x, y + length), 8)

    def on_click(self):
        facing_directions = [FacingDirection.UP, FacingDirection.RIGHT, FacingDirection.DOWN, FacingDirection.LEFT]
        current_facing_direction = facing_directions.index(self.facing_direction)
        self.facing_direction = facing_directions[(current_facing_direction + 1)%4]

    def set_goal_cell(self, cell):
        self.goal_cell = cell
        return
    
    def remove_goal_cell(self):
        self.goal_cell = None
        return
        
