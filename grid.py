import constants as const
import pygame
import numpy as np
from cell import Cell

MARGIN = const.MARGIN

class Grid():
    def __init__(self, grid_row: int, grid_col: int, block_size: int, top_left_pos: tuple):
        self.size_x = grid_row
        self.size_y = grid_col
        self.block_size = block_size

        self.outer_margin_x_pixel = top_left_pos[0] # pixel from left
        self.outer_margin_y_pixel = top_left_pos[1] # pixel from top

        self.cells = np.empty((self.size_x, self.size_y), dtype=Cell)
        self.initialize_cells()

    def initialize_cells(self):
        for x in range(self.size_x):
            for y in range(self.size_y):
                self.cells[x][y] = Cell(x, y)

    def get_grid_surface(self):
        self.grid_surface = pygame.Surface(self.get_total_pixel_size())
        self.grid_surface.fill(const.BLACK)

         # Draw the cells in the grid
        for x in range(self.size_x):
            for y in range(self.size_y):
                cell = self.cells[x][y]
                color = const.WHITE
                cell_surface = pygame.Surface((self.block_size, self.block_size))
                cell_surface.fill(color)
                self.grid_surface.blit(cell_surface, ((MARGIN + self.block_size) * x + MARGIN, (MARGIN + self.block_size) * (self.size_y - 1 - y) + MARGIN))
        return self.grid_surface

    def get_total_pixel_size(self) -> tuple:
        size_x_pixel = self.size_x * (self.block_size + MARGIN) + MARGIN
        size_y_pixel = self.size_y * (self.block_size + MARGIN) + MARGIN
        return (size_x_pixel, size_y_pixel)
