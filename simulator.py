import pygame
import math
import constants as const
from grid import Grid
from controls import Controls
from Robot import Robot
from queue import PriorityQueue
from obstacle import Obstacle
from obstacle import FacingDirection
from Astar import Astar
import Astar

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
        self.goals = []
        self.maze = []
        self.q = None

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
                            self.grid.set_cell_as_goal(current_cell[0], current_cell[1], direction)
                            self.grid.set_cell_as_barrier(current_cell[0], current_cell[1])
                            self.q = self.find_distance()
                            #self.show_cell_statuses()
                            
                            click_count+=1
                            
                        elif event.button == 3: # RIGHT CLICK
                            self.grid.set_cell_as_normal(current_cell[0], current_cell[1])

                    elif self.controls.click_selected_button(pos): # CHECK BUTTONS
                        pass
            self.draw_grid()
            self.controls.draw_buttons()
            self.robot.draw_robot()
            self.maze = Astar.make_maze(self.grid)
            pygame.display.update()
        pygame.quit()

    def show_cell_statuses(self):
        for x in range(20):
            for y in range(20):
                print([x, y], "=", self.grid.cells[x][y].status)

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
        goal_cells = self.grid.goal_cells

        for i in range (len(goal_cells)):
            x = goal_cells[i].x
            y = goal_cells[i].y
            d = math.sqrt((x-self.robot.location[0])**2 + (y-self.robot.location[1])**2) 
            q.put(([goal_cells[i].x, goal_cells[i].y], d))
        
        print("Goal cells in order of distance:")
        i = 0
        while not q.empty():
            i+=1
            next_item = q.get()
            print(i, next_item)
        print()

        return q

    def on_start(self):
        end_points = []
        for i in range (len(self.grid.goal_cells)):
            end_points.append([self.grid.goal_cells[i].x, self.grid.goal_cells[i].y])
            print(end_points[i])
        current_start = (1,1)
        current_orientation = const.NORTH
        #print(self.maze)
        path = []
        i = 0
        while i < len(self.obs):
            astar = Astar(self.grid, current_start, end_points[i])
            astar.set_maze(self.maze)
            leg, current_orientation = astar.make_path(current_orientation)
            #leg = Astar(self.maze, None, current_start, end_points[i])
            current_start = end_points[i]
            path.append(leg)
            i += 1

        return path

