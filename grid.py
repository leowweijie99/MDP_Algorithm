import constants as const
import pygame
import numpy as np
from cell import Cell
from cell import CellStatus

MARGIN = const.MARGIN

class Grid():
    def __init__(self, grid_row: int, grid_col: int, block_size: int, top_left_pos: tuple):
        self.size_x = grid_row
        self.size_y = grid_col
        self.block_size = block_size

        self.outer_margin_x_pixel = top_left_pos[0] # pixel from left
        self.outer_margin_y_pixel = top_left_pos[1] # pixel from top

        self.grid_surface = pygame.Surface(self.get_total_pixel_size())
        
        self.obstacles = []

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
                cell_surface.fill(color)
                if cell.obstacle != None:
                    cell.obstacle.draw_obstacle(cell_surface, 0, 0)
                self.grid_surface.blit(cell_surface, (x_coord, y_coord))
        return self.grid_surface

    def get_traversible_cells(self):
        traversible_cells = []

        for x in range(self.size_x):
            for y in range(self.size_y):
                cell = self.cells[x][y]
                x_coord = (MARGIN + self.block_size) * x + MARGIN
                y_coord = (MARGIN + self.block_size) * (self.size_y - 1 - y) + MARGIN

                if cell.status == CellStatus.EMPTY or cell.status == CellStatus.GOAL:
                    cell = self.find_cell_clicked(x_coord, y_coord)
                    traversible_cells.append(cell)

        return traversible_cells

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

    def set_cell_as_obstacle(self, pos_x, pos_y):
        self.cells[pos_x][pos_y].set_obstacle()
        if self.cells[pos_x][pos_y].obstacle not in self.obstacles:
            self.obstacles.append(self.cells[pos_x][pos_y].obstacle)


    def set_cell_as_normal(self, pos_x, pos_y):
        if self.cells[pos_x][pos_y].obstacle in self.obstacles:
            self.obstacles.remove(self.cells[pos_x][pos_y].obstacle)
        self.cells[pos_x][pos_y].remove_obstacle()

    
    def set_cell_image_direction(self, pos_x, pos_y, count):
        direction = self.cells[pos_x][pos_y].set_image(count)
        return(direction)
    
    def get_cell(self, x, y):
        return self.cells[x][y]