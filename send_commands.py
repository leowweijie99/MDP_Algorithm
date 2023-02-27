import time

from Astar import Astar, make_maze
from Robot import RobotMoves
from simulator import Simulator
from helper import get_commands
from flask import Flask, request, jsonify
from flask_cors import CORS
from grid import Grid
import constants as const
from obstacle import FacingDirection
import math
from queue import PriorityQueue

app = Flask(__name__)
CORS(app)

@app.route('/status', methods = ['GET'])
def status():
    return jsonify({"result": "ok"})

@app.route('/path', methods = ['POST'])
def path_maker():
    content = request.json

    d_to_facing_direction_map = {

        0: FacingDirection.UP,
        2: FacingDirection.RIGHT,
        4: FacingDirection.DOWN,
        6: FacingDirection.LEFT
    }

    grid = Grid(20, 20, const.BLOCK_SIZE, (0,0))
    for obstacle in obstacles:
        print('current obstacle is: ', obstacle)
        print('current directions orientation is: ', d_to_facing_direction_map[obstacle['d']])
        grid.set_cell_as_obstacle(obstacle['x'], obstacle['y'], obstacle['id'], d_to_facing_direction_map[obstacle['d']])
        grid.set_cell_as_goal(obstacle['x'], obstacle['y'], obstacle['id'], d_to_facing_direction_map[obstacle['d']])
        grid.set_cell_as_barrier(obstacle['x'], obstacle['y'])

    

    maze = make_maze(grid)
    print('goal cells are: ', grid.goal_cells)
    goal_cells = grid.goal_cells
    q = PriorityQueue()
    current_start = (1, 1)
    current_orientation = const.NORTH 
    for i in range (len(goal_cells)):
        print('current goal cell is: ', goal_cells[i].data)
        x = goal_cells[i].x
        y = goal_cells[i].y
        d = math.sqrt((x-1)**2 + (y-1)**2) 
        q.put((d, [goal_cells[i].x, goal_cells[i].y, goal_cells[i].facing_direction, goal_cells[i].id]))
    
    euclidean_q = []
    next_start = [1, 1]
    i = 0
    while (len(goal_cells)) > 0:
        node = get_closest_goal(goal_cells, next_start)
        euclidean_q.append(node)
        next_start = [euclidean_q[i][1][0], euclidean_q[i][1][1]]
        goal_cells.remove(node[2])

    end_points = []
    i = 0
    while (len(euclidean_q)) > 0: #len(self.grid.goal_cells) > 0:
        temp_point = euclidean_q[0]
        euclidean_q.remove(euclidean_q[0])
        end_points.append([temp_point[1][0], temp_point[1][1], temp_point[1][2], temp_point[1][3]])

    current_start = (1,1)
    current_orientation = const.NORTH
    path = []
    superpath = []
    i = 0
    to_execute = []

    i = 0
    while i < len(end_points):
        print(str(current_start) + ' ' + str(current_orientation))
        current_endpoint = (end_points[i][0], end_points[i][1])
        astar = Astar(grid, current_start, current_endpoint)
        astar.set_maze(maze)
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
    print(path)
    for leg in path:
        for movement in leg:
            superpath.append(movement)
        superpath.append([end_points[i][3]])
        i += 1

    commands = get_commands(superpath)

    return jsonify({
        "data": {'commands': commands},
        "error": None
    })

def get_closest_goal( goal_cells, next_start):

    q = PriorityQueue()
    for i in range (len(goal_cells)):
            x = goal_cells[i].x
            y = goal_cells[i].y
            d = math.sqrt((x-next_start[0])**2 + (y-next_start[1])**2)
            q.put((d, [goal_cells[i].x, goal_cells[i].y, goal_cells[i].facing_direction, goal_cells[i].id], goal_cells[i]))

    next_item = q.get()
    return next_item

def path(obstacles: list):

    d_to_facing_direction_map = {

        0: FacingDirection.UP,
        2: FacingDirection.RIGHT,
        4: FacingDirection.DOWN,
        6: FacingDirection.LEFT
    }

    grid = Grid(20, 20, const.BLOCK_SIZE, (0,0))
    for obstacle in obstacles:
        print('current obstacle is: ', obstacle)
        print('current directions orientation is: ', d_to_facing_direction_map[obstacle['d']])
        grid.set_cell_as_obstacle(obstacle['x'], obstacle['y'], obstacle['id'], d_to_facing_direction_map[obstacle['d']])
        grid.set_cell_as_goal(obstacle['x'], obstacle['y'], obstacle['id'], d_to_facing_direction_map[obstacle['d']])
        grid.set_cell_as_barrier(obstacle['x'], obstacle['y'])

    

    maze = make_maze(grid)
    print('goal cells are: ', grid.goal_cells)
    goal_cells = grid.goal_cells
    q = PriorityQueue()
    current_start = (1, 1)
    current_orientation = const.NORTH 
    for i in range (len(goal_cells)):
        print('current goal cell is: ', goal_cells[i].data)
        x = goal_cells[i].x
        y = goal_cells[i].y
        d = math.sqrt((x-1)**2 + (y-1)**2) 
        q.put((d, [goal_cells[i].x, goal_cells[i].y, goal_cells[i].facing_direction, goal_cells[i].id]))
    
    euclidean_q = []
    next_start = [1, 1]
    i = 0
    while (len(goal_cells)) > 0:
        node = get_closest_goal(goal_cells, next_start)
        euclidean_q.append(node)
        next_start = [euclidean_q[i][1][0], euclidean_q[i][1][1]]
        goal_cells.remove(node[2])
        i += 1

    end_points = []
    i = 0
    while (len(euclidean_q)) > 0: #len(self.grid.goal_cells) > 0:
        temp_point = euclidean_q[0]
        euclidean_q.remove(euclidean_q[0])
        end_points.append([temp_point[1][0], temp_point[1][1], temp_point[1][2], temp_point[1][3]])

    current_start = (1,1)
    current_orientation = const.NORTH
    path = []
    superpath = []
    i = 0
    to_execute = []

    i = 0
    while i < len(end_points):
        print(str(current_start) + ' ' + str(current_orientation))
        current_endpoint = (end_points[i][0], end_points[i][1])
        astar = Astar(grid, current_start, current_endpoint)
        astar.set_maze(maze)
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
    print(path)
    for leg in path:
        for movement in leg:
            superpath.append(movement)
        superpath.append([end_points[i][3]])
        i += 1

    commands = get_commands(superpath)
    return commands

obstacles = [
    {'x': 0, 'y': 15, 'id': 1, 'd': 4},
    {'x': 3, 'y': 19, 'id': 2, 'd': 4},
    {'x': 19, 'y': 19,'id': 3, 'd': 6},
    {'x': 13, 'y': 13, 'id': 4, 'd': 0},
    {'x': 13, 'y': 12, 'id': 5, 'd': 4},
    {'x': 9, 'y': 14, 'id': 6, 'd': 0}
]

print(obstacles)
print(path(obstacles))


if __name__ == '__send_commands__':
    app.run(host='0.0.0.0', port=5000, debug=True)
    

    
