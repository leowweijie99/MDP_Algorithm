import constants as const
import pygame
import numpy as np
from cell import Cell
from cell import CellStatus
from obstacle import Obstacle
from enum import IntEnum

MARGIN = const.MARGIN

class FacingDirection(IntEnum):
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3

class Grid():
    def __init__(self, grid_row: int, grid_col: int, block_size: int, top_left_pos: tuple):
        self.size_x = grid_row
        self.size_y = grid_col
        self.block_size = block_size

        self.outer_margin_x_pixel = top_left_pos[0] # pixel from left
        self.outer_margin_y_pixel = top_left_pos[1] # pixel from top

        self.grid_surface = pygame.Surface(self.get_total_pixel_size())
        
        self.obstacles = []
        self.goal_cells = []
        self.barrier_cells = []

        self.cells = np.empty((self.size_x, self.size_y), dtype=Cell)
        self.initialize_cells()

    def initialize_cells(self):
        for x in range(self.size_x):
            for y in range(self.size_y):
                self.cells[x][y] = Cell(x, y)

    def get_grid_surface(self):
        self.grid_surface.fill(const.BLACK)

         # Draw the cells in the grid
        for x in range(self.size_x):
            for y in range(self.size_y):
                cell = self.cells[x][y]
                x_coord = (MARGIN + self.block_size) * x + MARGIN
                y_coord = (MARGIN + self.block_size) * (self.size_y - 1 - y) + MARGIN
                cell_surface = pygame.Surface((self.block_size, self.block_size))
                if cell.status == CellStatus.OBS:
                    color = const.NAVY_BLUE
                if cell.status == CellStatus.GOAL:
                    color = const.RED
                if cell.status == CellStatus.BARRIER:
                    color = const.GREY
                if cell.status == CellStatus.EMPTY:
                    color = const.WHITE
                if cell.status == CellStatus.VISITED_GOAL:
                    color = const.GREEN
                cell_surface.fill(color)
                if cell.obstacle != None:
                    cell.obstacle.draw_obstacle(cell_surface, 0, 0)
                self.grid_surface.blit(cell_surface, (x_coord, y_coord))
        return self.grid_surface

    def get_total_pixel_size(self) -> tuple:
        size_x_pixel = self.size_x * (self.block_size + MARGIN) + MARGIN
        size_y_pixel = self.size_y * (self.block_size + MARGIN) + MARGIN
        return (size_x_pixel, size_y_pixel)

    def is_inside_grid(self, pos_x, pos_y):
        offset = const.HORIZONTAL_OFFSET
        vertical_offset = const.VERTICAL_OFFSET
        if pos_x <= offset + const.GRID_SIZE and pos_x >= offset and pos_y <= vertical_offset + const.GRID_SIZE and pos_y >= vertical_offset:
            return True
        return False

    def find_cell_clicked(self, pos_x, pos_y):
        if self.is_inside_grid(pos_x, pos_y):
            adjusted_pos_x = pos_x - const.HORIZONTAL_OFFSET
            adjusted_pos_y = pos_y - const.VERTICAL_OFFSET
            grid_x =  adjusted_pos_x//(const.BLOCK_SIZE + MARGIN)
            grid_y =  adjusted_pos_y//(const.BLOCK_SIZE + MARGIN)
            return [int(grid_x), int(19 - grid_y)]

        return None

    def get_pixel_measure(self, grid_x, grid_y):
        pix_x = const.HORIZONTAL_OFFSET + grid_x*(const.BLOCK_SIZE + MARGIN)
        pix_y = const.VERTICAL_OFFSET + grid_y*(const.BLOCK_SIZE + MARGIN)
        return [pix_x, pix_y]

    def set_cell_as_obstacle(self, pos_x, pos_y, id = -1, direction = FacingDirection.RIGHT):
        self.cells[pos_x][pos_y].set_obstacle(id, direction)
        if self.cells[pos_x][pos_y].obstacle not in self.obstacles:
            self.obstacles.append(self.cells[pos_x][pos_y].obstacle)
        print("Obstacles are:")
        for i in range (len(self.obstacles)):
            print(i+1, [self.obstacles[i].x, self.obstacles[i].y])
        print()

    def set_cell_as_goal(self, pos_x, pos_y, direction):
        obs_cell = self.get_cell(pos_x, pos_y)
        # Return if cell clicked is not a obstacle
        if (obs_cell.status != CellStatus.OBS):
            return
        
        goal_x, goal_y = pos_x, pos_y
        if direction == FacingDirection.UP:
            goal_y += 3
            orientation = const.SOUTH
        elif direction == FacingDirection.RIGHT:
            goal_x += 3
            orientation = const.WEST
        elif direction == FacingDirection.DOWN:
            goal_y -= 3
            orientation = const.NORTH
        elif direction == FacingDirection.LEFT:
            goal_x -= 3
            orientation = const.EAST

        # Setting the cell as a goal cell

        if(goal_x>20 or goal_x <=-1 or goal_y>20 or goal_y<=-1):
            print("Goal out of bounds")
            self.remove_goal(obs_cell)
        else:
            goal_cell = self.get_cell(goal_x, goal_y)
            goal_cell.set_goal(orientation)
            self.goal_cells.append(goal_cell.goal)
            if (obs_cell.obstacle.goal_cell != None):
                try:
                    self.remove_goal(obs_cell)
                except:
                    print()
            obs_cell.obstacle.set_goal_cell(goal_cell)
        
    def set_cell_as_barrier(self, pos_x, pos_y):
        o_pos = [pos_x, pos_y]
        for x in range(-2, 3):
            for y in range(-2, 3):
                b_pos = [pos_x-x, pos_y-y]
                if(b_pos[0]<20 and b_pos[0]>-1 and b_pos[1]<20 and b_pos[1]>-1):
                    cell_diff = abs(o_pos[0] - b_pos[0]) + abs(o_pos[1] - b_pos[1]) # Get the distance between current cell & the obstacle
                    if ( cell_diff < 4 and cell_diff > 0): # Corners diff is 4, Obstacle itself diff is 0
                        c = self.get_cell(b_pos[0], b_pos[1])
                        if(c.status != CellStatus.BARRIER):
                            c.set_barrier()
                            self.barrier_cells.append(c)

    def set_cell_as_normal(self, pos_x, pos_y):
        obs_cell = self.get_cell(pos_x, pos_y)
        if (obs_cell.status != CellStatus.OBS):
            return        

        o_pos = [pos_x, pos_y]
        # Remove Barriers

        try:
            for x in range(-2, 3):
                for y in range(-2, 3):
                    b_pos = [pos_x-x, pos_y-y]
                    if(b_pos[0]<20 and b_pos[0]>-1 and b_pos[1]<20 and b_pos[1]>-1):
                        cell_diff = abs(o_pos[0] - b_pos[0]) + abs(o_pos[1] - b_pos[1]) # Get the distance between current cell & the obstacl
                        if ( cell_diff < 4 and cell_diff > 0): # Corners diff is 4, Obstacle itself diff is 0
                            c = self.get_cell(b_pos[0], b_pos[1])
                            c.set_normal()
                            self.barrier_cells.remove(c)
        except:
            pass

        # Remove Goal
        #self.remove_goal(obs_cell)

        # Remove Obstacle
        self.obstacles.remove(obs_cell.obstacle)
        obs_cell.obstacle = None
        obs_cell.set_normal()

    
    def remove_goal(self, obstacle_cell):
        goal_cell = obstacle_cell.obstacle.goal_cell
        try:
            self.goal_cells.remove(goal_cell.goal)
            goal_cell.remove_goal()
            goal_cell.set_normal()
        except:
            pass


    def set_cell_image_direction(self, pos_x, pos_y, count):
        direction = self.cells[pos_x][pos_y].set_image(count)
        return(direction)
    
    def get_cell(self, x, y):
        return self.cells[x][y]

    def print_all(self):
        print("Obstacles are:")
        for i in range (len(self.obstacles)):
            print(i+1, [self.obstacles[i].x, self.obstacles[i].y])
        print()

        print("Goal Cells are:")
        for i in range (len(self.goal_cells)):
            print(i+1, [self.goal_cells[i].x, self.goal_cells[i].y])
        print()

        print("Barrier Cells are:")
        for i in range (len(self.barrier_cells)):
            print(i+1, [self.barrier_cells[i].x_coordinate, self.barrier_cells[i].y_coordinate])
        print()
        
               
    def set_cell_image_direction(self, pos_x, pos_y, count):
        direction = self.cells[pos_x][pos_y].set_image(count)
        return(direction)
    
    def get_cell(self, x, y):
        if (x > 19):
            x = x % 20
        if (y > 19):
            y = y % 20
        return self.cells[x][y]
            