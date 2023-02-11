from Robot import Robot
from obstacle import Obstacle, FacingDirection
import constants as const
from cell import CellStatus
from enum import IntEnum
from grid import Grid
from Robot import RobotMoves
import numpy as np
from queue import PriorityQueue

class Node():

    def __init__(self, grid, orientation: int = None, parent = None, position: tuple = None):
        self.grid = grid
        self.parent = parent
        self.position = position
        self.orientation = orientation
        self.children: list = []
        self.parent_to_child_move = None
        self.move_to_child = None
        #self.children = self.find_children()

        self.g = 0
        self.h = 0
        self.f = 0
        #self.cell = self.grid.cells[self.position[0], self.position[1]]

        self.move_to_parent = None


    def __eq__(self, other):
        return self.position == other.position

class Astar:
    def __init__(self, grid: Grid, start: tuple, end: tuple):

        self.start = start
        self.end = end
        self.TURNING_RADIUS = const.TURNING_RADIUS
        self.grid = grid

        self.robot_movement_mapping = {
            RobotMoves.FORWARD: [0,1],
            RobotMoves.BACKWARD: [0, -1],
            RobotMoves.FORWARD_LEFT: [-self.TURNING_RADIUS, self.TURNING_RADIUS],
            RobotMoves.BACKWARD_LEFT: [-self.TURNING_RADIUS, -self.TURNING_RADIUS],
            RobotMoves.FORWARD_RIGHT: [self.TURNING_RADIUS, self.TURNING_RADIUS],
            RobotMoves.BACKWARD_RIGHT: [self.TURNING_RADIUS, -self.TURNING_RADIUS]
        }

        self.maze_obstacles = []
        self.maze = []

        self.robot_movement_orientation_mapping = {
            RobotMoves.FORWARD: const.NORTH,
            RobotMoves.BACKWARD: const.NORTH,
            RobotMoves.FORWARD_LEFT: const.WEST,
            RobotMoves.FORWARD_RIGHT: const.EAST
        }

        self.statuses = [CellStatus.OBS, CellStatus.VISITED_OBS, CellStatus.BOUNDARY]

        self.turn_types = [RobotMoves.BACKWARD_LEFT, RobotMoves.BACKWARD_RIGHT, RobotMoves.FORWARD_LEFT, RobotMoves.FORWARD_LEFT]

        self.rotation_matrices = {
            const.NORTH: np.array([[1,0],
                                  [0,1]]),
            const.SOUTH: np.array([[-1,0],
                                  [0,-1]]),
            const.EAST: np.array([[0,1],
                                 [-1,0]]),
            const.WEST: np.array([[0,-1],
                                 [1,0]])
        }

        self.relative_direction_vectors = {
            const.NORTH: np.array([0, 1]),
            const.SOUTH: np.array([0, -1]),
            const.EAST: np.array([1, 0]),
            const.WEST: np.array([-1, 0]),
        }

        self.turn_cost = 3
        self.straight_cost = 1

        self.forward_vectors = {
            const.NORTH: np.array([0,1]),
            const.SOUTH: np.array([0, -1]),
            const.EAST: np.array([1, 0]),
            const.WEST: np.array([-1, 0]),
        }

        self.antagonistic_pairs = {
            FacingDirection.UP: const.SOUTH,
            FacingDirection.DOWN: const.NORTH,
            FacingDirection.LEFT: const.EAST,
            FacingDirection.RIGHT: const.WEST
        }

    def set_maze(self, maze):
        self.maze = maze

    def make_path(self, start_orientation):

        end_orientation = self.get_expected_goal_orientation()

        self.start = Node(self.grid, start_orientation, None, self.start)
        end_node = Node(self.grid, end_orientation, None, self.end)

        self.maze_obstacles = self.grid.obstacles

        unvisited = []
        visited = []

        unvisited.append(self.start)
        
        while len(unvisited) > 0:
            current_node: Node = unvisited[0]
            current_index = 0
            for index, item in enumerate(unvisited):
                if item.f<current_node.f:
                    current_node = item
                    current_index = index

            unvisited.pop(current_index)
            visited.append(current_node)

            if current_node == end_node and self.correct_orientation(current_node.orientation):
                return self.populate_path(current_node), current_node.orientation

            current_node_children = self.get_children(current_node)

            child: Node
            for child in current_node_children:
                if child not in visited:
                    continue

                child.g = self.get_cost(child)
                child.h = self.heuristic(child)
                child.f = child.g + child.h
                unvisited.append(child)
                unvisited = self.sort_unvisited_set(unvisited)

    def sort_unvisited_set(self, node_list: list, node) -> list:
        for index, item in enumerate(node_list):
            if item.f<node.f:
                node = item
                current_index = index

        node_list[0] = node
        node_list.pop(current_index)

        return node_list

    def populate_path(self, node: Node) -> list:
        node_path = []
        movements = []
        current_node = node

        while current_node != None:
            node_path.insert(0, current_node)
            movements.insert(0, current_node.parent_to_child_move)
            current_node = current_node.parent

        return movements, node_path

    def within_boundary(self, position: tuple):
        if position[0] > const.NUM_OF_BLOCKS - 1 or position[0] < 0 or position[1] > const.NUM_OF_BLOCKS-1 or position[1] < 0:
            return True
        return False

    def get_absolute_vector(self, relative_vector: list, direction):
        rotation_matrix = self.rotation_matrices[direction]
        return np.matmul(rotation_matrix, relative_vector).astype(int)

    def get_absolute_direction(self, relative_direction, current_direction):
        resultant_direction = relative_direction + current_direction
        if resultant_direction > 180:
            return resultant_direction - 360
        if resultant_direction <= -180:
            return resultant_direction + 360
        return resultant_direction

    def resultant_child_node(self, move: RobotMoves, node: Node):
        new_relative_pos = self.robot_movement_mapping[move]
        actual_pos = self.get_absolute_vector(new_relative_pos, node.orientation)
        relative_direction_from_node = self.robot_movement_orientation_mapping[move]

        node_position = (
            node.position[0] + actual_pos[0],
            node.position[1] + actual_pos[1],)

        node_orientation = self.get_absolute_direction(relative_direction_from_node, node.orientation)

        child_node = Node(self.grid, node_orientation, node, node_position)
        child_node.parent_to_child_move = move

        return child_node

    def get_children(self, node: Node):
        
        for movement in self.robot_movement_mapping:

            child = self.resultant_child_node(movement, node)

            if not self.within_boundary(child):
                continue

            if not self.reachable(child):
                continue
            
            node.children.append(child)


    def reachable(self, node: Node) -> bool:
        return self.path_leads_to_collision(node)

    def path_leads_to_collision(self, node: Node):
        if self.within_boundary(node):
            return self.maze_pos_status(node.position) in self.statuses

    def maze_pos_status(self, position: tuple) -> CellStatus:
        if self.maze[position[0]][position[1]] == 1:
            return CellStatus.OBS
        if self.maze[position[0]][position[1]] == 2: 
            return CellStatus.BARRIER
        if self.maze[position[0]][position[1]] == 3:
            return CellStatus.VISITED_OBS

    def heuristic(self, node: Node):
        xpos, ypos = node.position[0], node.position[1]
        end_xpos, end_ypos = self.end[0], self.end[1]
        return abs(end_xpos - xpos) + abs(end_ypos - ypos)

    def get_cost(self, node: Node):
        straight_cost = 3
        move = node.parent_to_child_move

        if move in self.turn_types:
            cost = straight_cost*self.turn_cost
        else:
            cost = straight_cost
        
        return cost

    def get_expected_goal_orientation(self) -> int:
        obs_orientation = self.grid.cells[self.end[0]][self.end[1]].obstacle.facing_direction
        return self.antagonistic_pairs(obs_orientation)

    def correct_orientation(self, orientation) -> bool:
        obs_orientation = self.grid.cells[self.end[0]][self.end[1]].obstacle.facing_direction

        return orientation == self.antagonistic_pairs(obs_orientation)
        








    
'''def Astar(maze, grid: Grid, start, end, robot: Robot = None):

    start_node = Node(grid, None, None , start)
    start_node.g = start_node.h = start_node.f = 0

    

    end_node = Node(grid, None, None, end)
    end_node.g = end_node.h = end_node.f = 0

    #obstacle = end_node.cell.obstacle

    open_list = []
    closed_list = []

    open_list.append(start_node)

    while len(open_list) > 0:
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f<current_node.f:
                current_node = item
                current_index = index

        open_list.pop(current_index)
        closed_list.append(current_node)

        if current_node == end_node: #and is_in_opposition(robot, obstacle):
            path = []
            current = current_node

            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]
        elif current_node == end_node: #and not is_in_opposition(robot, obstacle):
            continue


        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            if node_position[0] >(len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1])-1) or node_position[1] < 0:
                continue

            if maze[node_position[0]][node_position[1]] != 0:
                continue

            new_node = Node(grid, None, current_node, node_position)

            children.append(new_node)

            for child in children:

                for closed_child in closed_list:
                    if child == closed_child:
                        continue

                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue
                
                open_list.append(child)
'''
def make_maze(grid):
    maze = []
    for x in range(const.NUM_OF_BLOCKS):
        row = []
        for y in range(const.NUM_OF_BLOCKS):
            if grid.cells[x][y].obstacle == None:
                row.append(0)
            elif grid.cells[x][y].status == CellStatus.BARRIER:
                row.append(2)
            else:
                row.append(1)
        maze.append(row)

    return maze

def is_in_opposition(robot, obstacle):
    antagonistic_pairs = [("NORTH", FacingDirection.DOWN), ("SOUTH", FacingDirection.UP), 
                            ("EAST", FacingDirection.LEFT), ("WEST", FacingDirection.RIGHT)]

    input_pair = (robot.get_direction(), obstacle.facing_direction)

    if input_pair in antagonistic_pairs:
        return True
    else:
        return False


def main():
    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]]

    start = (0, 0)
    end = (7, 6)

    path = Astar(maze, None, start, end, None)
    print(path)

#main()

def unit_test():

    maze = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 2, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
    start = (0,0)
    goal = (7,4)

    grid = Grid(20, 10, const.BLOCK_SIZE, (0,0))

    grid.obstacles = [Obstacle(4, 4, FacingDirection.RIGHT)]

    astar = Astar(grid, start, goal)

    astar.set_maze(maze)

    path = astar.make_path(const.NORTH)
    print(path)


#unit_test()