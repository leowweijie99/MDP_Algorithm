from Robot import Robot
from obstacle import Obstacle, FacingDirection
import constants as const
from cell import CellStatus
from enum import IntEnum
from grid import Grid
from Robot import RobotMoves
import numpy as np
from queue import PriorityQueue

class CellData():

    def __init__(self, position: tuple, orientation: int, status: CellStatus = CellStatus.EMPTY):
        self.data = (position[0], position[1], orientation)
        self.status = status

class Node():

    def __init__(self, grid, orientation: int = None, parent = None, position: tuple = None):
        self.grid = grid
        self.parent = parent
        self.position = position
        self.orientation = orientation
        self.children: list = []
        self.parent_to_child_move = None
        self.move_to_child = None
        self.displacement_from_parent: np.ndarray()
        self.data = (self.position, self.orientation)
        #self.children = self.find_children()

        self.g = 0
        self.h = 0
        self.f = 0
        self.cell = self.grid.cells[self.position[0], self.position[1]]

        self.move_to_parent = None

    def __lt__(self, other):
        return other

class Bstar:
    def __init__(self, grid: Grid, start: tuple, end: tuple):

        self.start = start
        self.end = end
        self.TURNING_RADIUS = const.TURNING_RADIUS
        self.grid = grid
        self.safe_squared_obstacle_distance = 8
        self.goal_node_set = []
        self.additional_car_nodes = []

        self.robot_movement_mapping = {
            RobotMoves.FORWARD: [0,1],
            RobotMoves.BACKWARD: [0, -1],
            RobotMoves.FORWARD_LEFT: [-self.TURNING_RADIUS, 2],
            RobotMoves.BACKWARD_LEFT: [-self.TURNING_RADIUS, -2],
            RobotMoves.FORWARD_RIGHT: [self.TURNING_RADIUS, 2],
            RobotMoves.BACKWARD_RIGHT: [self.TURNING_RADIUS, -2]
        }

        self.maze_obstacles = []
        self.maze = []

        self.robot_movement_orientation_mapping = {
            RobotMoves.FORWARD: const.NORTH,
            RobotMoves.BACKWARD: const.NORTH,
            RobotMoves.FORWARD_LEFT: const.WEST,
            RobotMoves.FORWARD_RIGHT: const.EAST,
            RobotMoves.BACKWARD_RIGHT: const.WEST,
            RobotMoves.BACKWARD_LEFT: const.EAST
        }

        self.statuses = [CellStatus.OBS, CellStatus.VISITED_OBS, CellStatus.BOUNDARY]

        self.turn_types = [RobotMoves.BACKWARD_LEFT, RobotMoves.BACKWARD_RIGHT, RobotMoves.FORWARD_LEFT, RobotMoves.FORWARD_RIGHT]

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

        self.turn_cost = 5
        self.straight_cost = 5

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

        self.respective_forward_vector = {
            const.NORTH: np.array([0, 1]),
            const.SOUTH: np.array([0, -1]),
            const.EAST: np.array([1, 0]),
            const.WEST: np.array([-1, 0]),
        }

    def set_maze(self, maze):
        self.maze = maze
        self.maze_width = len(maze[0])
        self.maze_height = len(maze)
        self.set_obstacles()

    def set_obstacles(self):
        for i in range(self.maze_height):
            for j in range(self.maze_width):
                if self.maze[i][j] == 0:
                    self.maze_obstacles.append([i, j])

    def is_unreachable_from_parent(self, node: Node):
        if not self.reachable(node):
            return True

        parent: Node = node.parent
        child: Node = node
        displacement_from_parent = child.displacement_from_parent

        if node.parent_to_child_move in self.turn_types:
            unit_forward_vector = self.forward_vectors[parent.orientation]
            corner_displacement = np.matmul(unit_forward_vector, displacement_from_parent) * unit_forward_vector
            corner_position = (corner_displacement[0] + parent.position[0], corner_displacement[1] + parent.position[1])
            intermediate_node = Node(self.grid, const.NORTH, None, corner_position)
            if not self.reachable(intermediate_node):
                return True
        
        return False

    def make_path(self, start_orientation, end_orientation):

        self.start = Node(self.grid, start_orientation, None, self.start)
        end_node = Node(self.grid, end_orientation, None, self.end)

        self.goal_node_set.append(end_node.data)
        
        self.set_adjacent_squares_to_goals(end_node)

        unvisited = PriorityQueue()
        visited = []

        unvisited.put((0, self.start))
        
        while not unvisited.empty():
            current_node: Node = unvisited.get()[1]
            visited.append(current_node.data)

            # This function checks if the current_node is in range of the goal node, each node has an orientation to determine the position of the car?!
            frontier_node: Node = self.get_robot_frontier(current_node)
            if frontier_node.data in self.goal_node_set:
                path = self.populate_path(current_node)
                return path, current_node.position

            # This function returns the possible nodes the car can move to
            self.get_children(current_node)

            # This function sets the heuristics and appends it to the unvisited list
            child: Node
            for child in current_node.children:
                if child.data in visited:
                    continue
                child.g = current_node.g + self.get_cost(child)
                child.h = self.heuristic(child)
                child.f = child.g + child.h
                unvisited.put((child.f, child))

    def populate_path(self, node: Node) -> list:
        node_path = []
        movements = []
        current_node = node
        
        while current_node != None:
            current_node.cell.status = CellStatus.PATH
            node_path.insert(0, current_node)
            movements.insert(0, current_node.parent_to_child_move)
            current_node = current_node.parent

        return movements

    def within_boundary(self, position: tuple):
        if position[0] - 1 < 0 or position[0] + 1 > self.maze_width-1 or position[1] + 1  > self.maze_height-1 or position[1] -1 < 0:
            return False

        return True

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
            node.position[1] + actual_pos[1])

        if not self.within_boundary(node_position):
            return None

        node_orientation = self.get_absolute_direction(relative_direction_from_node, node.orientation)

        child_node = Node(self.grid, node_orientation, node, node_position)
        child_node.parent_to_child_move = move
        child_node.displacement_from_parent = actual_pos

        return child_node

    def get_children(self, node: Node) -> list:
        for movement in self.robot_movement_mapping:

            child = self.resultant_child_node(movement, node)

            if child == None:
                continue

            if self.is_unreachable_from_parent(child):
                continue

            node.children.append(child)

    def reachable(self, node: Node) -> bool:
        if not self.within_boundary:
            return False
        
        if self.path_leads_to_collision(node):
            return False

        for obstacle in self.maze_obstacles:
            current_distance = self.squared_euclidean_distance(node.position[0], node.position[1], obstacle[0], obstacle[1])
            if current_distance < self.safe_squared_obstacle_distance:
                return False
        
        return True

    def path_leads_to_collision(self, node: Node):
            return self.maze_pos_status(node.position) in self.statuses

    def maze_pos_status(self, position: tuple) -> CellStatus:
            if self.maze[position[1]][position[0]] == 0:
                return CellStatus.OBS
            if self.maze[position[1]][position[0]] == 2: 
                return CellStatus.BARRIER
            if self.maze[position[1]][position[0]] == 3:
                return CellStatus.VISITED_OBS
            else:
                return CellStatus.EMPTY

    def heuristic(self, node: Node):
        xpos, ypos = node.position[0], node.position[1]
        end_xpos, end_ypos = self.end[0], self.end[1]
        return abs(end_xpos - xpos) + abs(end_ypos - ypos)

    def squared_euclidean_distance(self, pos1x, pos1y, pos2x, pos2y):
        return (pos2x-pos1x)**2 + (pos2y-pos1y)**2

    def set_adjacent_squares_to_goals(self, node: Node):
        key_goal_data = node.data
        xpos, ypos = key_goal_data[0]
        additional_targets = []

        if key_goal_data[1] == const.NORTH:
            additional_targets.append(((xpos-1, ypos), const.NORTH))
            additional_targets.append(((xpos+1, ypos), const.NORTH))
        elif key_goal_data[1] == const.SOUTH:
            additional_targets.append(((xpos-1, ypos), const.SOUTH))
            additional_targets.append(((xpos+1, ypos), const.SOUTH))
        elif key_goal_data[1] == const.EAST:
            additional_targets.append(((xpos, ypos+1), const.EAST))
            additional_targets.append(((xpos, ypos-1), const.EAST))
        else:
            additional_targets.append(((xpos, ypos+1), const.WEST))
            additional_targets.append(((xpos, ypos-1), const.WEST))

        for target in additional_targets:
            target_node = Node(self.grid, key_goal_data[1], None, target[0])
            if self.within_boundary(target[0]) and not self.path_leads_to_collision(target_node):
                self.goal_node_set.append(target)

    def get_cost(self, node: Node):
        straight_cost = 3
        move = node.parent_to_child_move

        if move in self.turn_types:
            cost = straight_cost*self.turn_cost
        else:
            cost = straight_cost

        return cost

    def get_robot_frontier(self, node: Node)-> list:
        frontier_node: Node
        xpos = node.position[0]
        ypos = node.position[1]

        if node.orientation == const.NORTH:
            frontier_node = Node(self.grid, const.NORTH, None, (xpos, ypos+1))
        elif node.orientation == const.SOUTH:
            frontier_node = Node(self.grid, const.SOUTH, None, (xpos, ypos-1))
        elif node.orientation == const.WEST:
            frontier_node = Node(self.grid, const.WEST, None, (xpos-1, ypos))
        elif node.orientation == const.EAST:
            frontier_node = Node(self.grid, const.EAST, None, (xpos+1, ypos))       

        return frontier_node

def correct_maze(maze: list):
    for i in range(len(maze)):
        for j in range(len(maze[i])):
            if maze[i][j] == 0:
                maze[i][j] = 1
            elif maze[i][j] == 1:
                maze[i][j] = 0