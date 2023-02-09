from Robot import Robot
from obstacle import FacingDirection
import constants as const
from cell import CellStatus

class Node():

    def __init__(self, grid, parent=None, position=None):
        self.grid = grid
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0
        #self.cell = self.grid.cells[self.position[0], self.position[1]]

    def __eq__(self, other):
        return self.position == other.position
    
def Astar(maze, grid, start, end, robot):

    start_node = Node(grid, None, start)
    start_node.g = start_node.h = start_node.f = 0

    end_node = Node(grid, None, end)
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

            new_node = Node(None, current_node, node_position)

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
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]]

    start = (0, 0)
    end = (7, 6)

    path = Astar(maze, None, start, end, None)
    print(path)

#main()