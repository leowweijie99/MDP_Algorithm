import pygame
import constants as const
from grid import Grid
from controls import Controls
from Robot import Robot

class Simulator:
    def __init__(self):
        pygame.init()
        pygame.font.init()
        pygame.display.set_caption("MDP Algorithm Simulator")
        self.screen = pygame.display.set_mode((const.WIDTH, const.HEIGHT))
        self.screen.fill(const.BLACK)
        self.grid_from_screen_top_left = ((const.WIDTH/2) - (const.GRID_SIZE/2), 50)
        self.grid = Grid(20, 20, const.BLOCK_SIZE, self.grid_from_screen_top_left)

        #Initialize Robot
        self.robot = Robot(self.screen, self.grid, 0)
        
        #Initialize Button Control Panel
        self.controls = Controls(self.screen, self.robot)

    def run(self):
        running = True
        clock = pygame.time.Clock()
        click_count = 0
        obs = []
        while (running):
            clock.tick(const.FPS)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        pos = pygame.mouse.get_pos()
                        if self.grid.is_inside_grid(pos[0], pos[1]):
                            current_cell = self.grid.find_cell_clicked(pos[0], pos[1])
                            self.grid.set_cell_as_obstacle(current_cell[0], current_cell[1])
                            direction = self.grid.set_cell_image_direction(current_cell[0], current_cell[1], click_count)
                            duplicate = 0
                            index = 0
                            for x in range (len(obs)):
                                if current_cell == obs[x][0]:
                                    duplicate = 1
                                    index = x
                            if duplicate == 1:
                                obs.remove(obs[index])
                                obs.append((current_cell, direction))
                            else:
                                obs.append((current_cell, direction))
                            print("Obstacles are = ", obs)
                            click_count+=1

                            
                    if event.button == 3:
                        pos = pygame.mouse.get_pos()
                        if self.grid.is_inside_grid(pos[0], pos[1]):
                            current_cell = self.grid.find_cell_clicked(pos[0], pos[1])
                            self.grid.set_cell_as_normal(current_cell[0], current_cell[1])
                            duplicate = 0
                            index = 0
                            for x in range (len(obs)):
                                if current_cell == obs[x][0]:
                                    duplicate = 1
                                    index = x
                                if duplicate == 1:
                                    obs.remove(obs[index])
                            print(obs)




                    elif self.controls.click_selected_button(pos):
                        pass
            self.draw_grid()
            self.controls.draw_buttons()
            self.robot.draw_robot()
            pygame.display.update()
        pygame.quit()

    

    def draw_grid(self):
        self.grid_surface = self.grid.get_grid_surface()
        self.screen.blit(self.grid_surface, self.grid_from_screen_top_left)
        start_pos = (self.grid_from_screen_top_left[0]-const.BLOCK_SIZE, self.grid_from_screen_top_left[1])
        font = pygame.font.SysFont('Futura', const.BLOCK_SIZE)

        for x in range(self.grid.size_x):
            index = font.render(str(19-x), False, (255, 255, 255))
            self.screen.blit(index, (start_pos[0], start_pos[1] + const.MARGIN + (x * (const.BLOCK_SIZE + const.MARGIN))))

        start_pos = (self.grid_from_screen_top_left[0], self.grid_from_screen_top_left[1] + const.GRID_SIZE)

        for y in range(self.grid.size_y):
            index = font.render(str(y), False, (255, 255, 255))
            self.screen.blit(index, (start_pos[0] + const.MARGIN + (y * (const.BLOCK_SIZE + const.MARGIN)), start_pos[1]))

