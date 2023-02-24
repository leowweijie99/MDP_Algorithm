import pygame
import math
import constants as const
from grid import Grid
from controls import Controls
from Robot import Robot, RobotMoves
from queue import PriorityQueue
from obstacle import Obstacle
from obstacle import FacingDirection
from Astar import Astar
from cell import CellStatus
from helper import get_commands

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
        self.q : PriorityQueue = PriorityQueue()

        self.clock = pygame.time.Clock()

        self.path = []

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
                            #self.grid.set_cell_as_goal(current_cell[0], current_cell[1], direction)
                            #self.grid.set_cell_as_barrier(current_cell[0], current_cell[1])
                            #self.q = self.find_distance()
                            #self.show_cell_statuses()
                            
                            click_count+=1
                            
                        elif event.button == 3: # RIGHT CLICK
                            self.grid.set_cell_as_normal(current_cell[0], current_cell[1])

                    elif self.controls.click_selected_button(pos): # CHECK BUTTONS
                        pass
            self.draw_grid()
            self.controls.draw_buttons()
            self.robot.draw_robot()
            self.maze = self.make_maze()
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
        goal_cells = self.grid.goal_cells
        self.q = PriorityQueue()

        for i in range (len(goal_cells)):
            x = goal_cells[i].x
            y = goal_cells[i].y
            d = math.sqrt((x-self.robot.location[0])**2 + (y-self.robot.location[1])**2) 
            self.q.put(([goal_cells[i].x, goal_cells[i].y, goal_cells[i].facing_direction], d))

    def get_closest(self, planning_current_position) -> tuple:

        goal_cells = self.grid.goal_cells
        index = 0
        q = PriorityQueue()
        for i in range(len(goal_cells)):
            d = math.sqrt((goal_cells[i].x-planning_current_position[0])**2 + (goal_cells[i].y-planning_current_position[1])**2)
            q.put((d, i, goal_cells[i]))

        closest_goal_cell = q.get()
        x = closest_goal_cell[2].x
        y = closest_goal_cell[2].y
        orientation = closest_goal_cell[2].facing_direction

        self.grid.goal_cells.remove(closest_goal_cell[2])


        return (x, y, orientation)

    def get_closest_goal(self, goal_cells, next_start):

        q = PriorityQueue()
        for i in range (len(goal_cells)):
            if(goal_cells[i] != None):
                x = goal_cells[i].x
                y = goal_cells[i].y
                d = math.sqrt((x-next_start[0])**2 + (y-next_start[1])**2)
                q.put((d, [goal_cells[i].x, goal_cells[i].y, goal_cells[i].facing_direction], goal_cells[i]))

        next_item = q.get()
        return next_item
    

    def on_start(self):
        tried = False
        for obstacle in self.grid.obstacles:
            self.grid.set_cell_as_goal(obstacle.x, obstacle.y, obstacle.facing_direction)
            self.grid.set_cell_as_barrier(obstacle.x, obstacle.y)

        goal_cells = self.grid.goal_cells
        q = PriorityQueue()
        for i in range (len(goal_cells)):
            if(goal_cells[i] != None):
                x = goal_cells[i].x
                y = goal_cells[i].y
                d = math.sqrt((x-self.robot.location[0])**2 + (y-self.robot.location[1])**2) 
                q.put((d, [goal_cells[i].x, goal_cells[i].y, goal_cells[i].facing_direction]))

        euclidean_q = []
        next_start = [1, 1]
        i = 0
        while (len(goal_cells)) > 0:
            node = self.get_closest_goal(goal_cells, next_start)
            euclidean_q.append(node)
            next_start = [euclidean_q[i][1][0], euclidean_q[i][1][1]]
            goal_cells.remove(node[2])


        end_points = []
        i = 0
        while (len(euclidean_q)) > 0: #len(self.grid.goal_cells) > 0:
            temp_point = euclidean_q[0]
            euclidean_q.remove(euclidean_q[0])
            end_points.append([temp_point[1][0], temp_point[1][1], temp_point[1][2]])
            i += 1
        current_start = (1,1)
        current_orientation = const.NORTH
        path = []
        superpath = []
        i = 0
        to_execute = []
        while i < len(end_points):
            print(str(current_start) + ' ' + str(current_orientation))
            current_endpoint = (end_points[i][0], end_points[i][1])
            astar = Astar(self.grid, current_start, current_endpoint)
            astar.set_maze(self.maze)
            tried = False
            try:
                leg, resultant_pos = astar.make_path(current_orientation, end_points[i][2])
                current_start = resultant_pos
                current_orientation = end_points[i][2]
                path.append(leg)
                to_execute.append(end_points[i])
                i += 1
            except:
                print("Path not found to ", end_points[i])
                i += 1

        i = 0
        for leg in path:
            for movement in leg:
                superpath.append(movement)
            superpath.append(to_execute[i])
            i += 1

        self.robot.movement_queue = superpath
        print(get_commands(superpath))
        return path

    def make_maze(self):
        maze = []
        for x in range(const.NUM_OF_BLOCKS):
            row = []
            for y in range(const.NUM_OF_BLOCKS):
                if self.grid.cells[x][y].status == CellStatus.OBS:
                    row.append(0)
                elif self.grid.cells[x][y].status == CellStatus.BARRIER:
                    row.append(2)
                elif self.grid.cells[x][y].status == CellStatus.VISITED_OBS:
                    row.append(3)
                else:
                    row.append(1)
            maze.append(row)

        return maze
