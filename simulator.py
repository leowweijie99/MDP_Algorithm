import pygame
import math
import constants as const
from grid import Grid
from controls import Controls
from robot import Robot
from queue import PriorityQueue
from obstacle import Obstacle
from obstacle import FacingDirection

class Simulator:
    def __init__(self):
        pygame.init()
        pygame.font.init()
        pygame.display.set_caption("MDP Algorithm Simulator")
        self.screen = pygame.display.set_mode((const.WIDTH, const.HEIGHT))
        self.screen.fill(const.BLACK)
        self.grid_from_screen_top_left = ((const.WIDTH/2) - (const.GRID_SIZE/2), 50)
        self.grid = Grid(20, 20, const.BLOCK_SIZE, self.grid_from_screen_top_left)
        self.obs = []

        #Initialize Robot
        self.robot = Robot(self.screen, self.grid, 0)
        
        #Initialize Button Control Panel
        self.controls = Controls(self.screen, self)

    def run(self):
        running = True
        clock = pygame.time.Clock()
        click_count = 0
        self.obs = []
        while (running):
            clock.tick(const.FPS)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()
                    if self.grid.is_inside_grid(pos[0], pos[1]): # CHECK IF CLICK IS INSIDE THE GRID
                        current_cell = self.grid.find_cell_clicked(pos[0], pos[1])
                        if event.button == 1: # LEFT CLICK
                            self.grid.set_cell_as_obstacle(current_cell[0], current_cell[1])
                            direction = self.grid.set_cell_image_direction(current_cell[0], current_cell[1], click_count)
                            duplicate = 0
                            index = 0
                            for x in range (len(self.obs)):
                                if current_cell == self.obs[x][0]:
                                    duplicate = 1
                                    index = x
                            if duplicate == 1:
                                self.obs.remove(self.obs[index])
                                self.obs.append((current_cell, direction))
                            else:
                                self.obs.append((current_cell, direction))
                            #self.print_obs()
                            self.find_distance()
                            click_count+=1
                        elif event.button == 3: # RIGHT CLICK
                            self.grid.set_cell_as_normal(current_cell[0], current_cell[1])
                            duplicate = 0
                            index = 0
                            for x in range (len(self.obs)):
                                if current_cell == self.obs[x][0]:
                                    duplicate = 1
                                    index = x
                            if duplicate == 1:
                                self.obs.remove(self.obs[index])
                            #self.print_obs()
                    elif self.controls.click_selected_button(pos): # CHECK BUTTONS
                        print(self.robot.location)
            self.screen.fill(const.BLACK)
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

    def print_obs(self):
        print("Obstacles are = ", self.obs)

    def get_obs(self):
        return self.obs

    def find_distance(self):
        q = PriorityQueue()

        for i in range (len(self.obs)):
            x = self.obs[i][0][0]
            y = self.obs[i][0][1]
            d = math.sqrt((x-self.robot.location[0])**2 + (y-self.robot.location[1])**2) 
            q.put((self.obs[i], d))
        
        while not q.empty():
            next_item = q.get()
            print(next_item)

        print()

        return q
    
    def find_goal_cells(self):
        obstacles = self.grid.obstacles
        goal_cells = []
        for o in obstacles:
            x = o.x
            y = o.y
            cell = None
            if o.facing_direction == FacingDirection.UP:
                cell = self.grid.get_cell(x, y-2)
            elif o.facing_direction == FacingDirection.RIGHT:
                cell = self.grid.get_cell(x+2, y)
            elif o.facing_direction == FacingDirection.DOWN:
                cell = self.grid.get_cell(x, y+2)
            else: # LEFT
                cell = self.grid.get_cell(x-2, y)
            cell.set_goal()
            goal_cells.append(cell)
        return goal_cells
