import pygame
import constants as const
from grid import Grid

class Simulator:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("MDP Algorithm Simulator")
        self.screen = pygame.display.set_mode((const.WIDTH, const.HEIGHT))
        self.screen.fill(const.BLACK)
        self.grid_from_screen_top_left = ((const.WIDTH/2) - (20*34-2)/2, 50)
        self.grid = Grid(20, 20, 32, self.grid_from_screen_top_left)

    def run(self):
        running = True
        clock = pygame.time.Clock()
        while (running):
            clock.tick(const.FPS)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            self.draw_grid()
            pygame.display.update()
        pygame.quit()
    

    def draw_grid(self):
        self.grid_surface = self.grid.get_grid_surface()
        self.screen.blit(self.grid_surface, self.grid_from_screen_top_left)
        print("drew grid")

