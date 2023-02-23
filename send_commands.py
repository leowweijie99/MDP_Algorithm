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

    print("content: {}".format(content))
    obstacles = content['obstacles']
    grid = Grid(20, 20, const.BLOCK_SIZE, (0,0))
    for obstacle in obstacles:
        grid.set_cell_as_obstacle(obstacle['x'], obstacle['y'], obstacle['id'], d_to_facing_direction_map[obstacle['d']])
        grid.set_cell_as_goal(obstacle['x'], obstacle['y'], d_to_facing_direction_map[obstacle['d']], obstacle['id'])
        grid.set_cell_as_barrier(obstacle['x'], obstacle['y'])

    

    maze = make_maze(grid)
    goal_cells = grid.goal_cells
    q = PriorityQueue()
    current_start = (1, 1)
    current_orientation = const.NORTH 
    for i in range (len(goal_cells)):
        x = goal_cells[i].x
        y = goal_cells[i].y
        d = math.sqrt((x-1)**2 + (y-1)**2) 
        q.put(([goal_cells[i].x, goal_cells[i].y, goal_cells[i].facing_direction, goal_cells[i].id], d))
    
    end_points = []
    i = 0
    while not q.empty(): #len(self.grid.goal_cells) > 0:
        temp_point = q.get()
        print(temp_point)
        end_points.append([temp_point[0][0], temp_point[0][1], temp_point[0][2], temp_point[0][3]])
        print(i+1 , str(end_points[i]))
        i += 1

    path = []
    superpath = []

    i = 0
    while i < len(end_points):
        print(str(current_start) + ' ' + str(current_orientation))
        current_endpoint = (end_points[i][0], end_points[i][1])
        astar = Astar(grid, current_start, current_endpoint)
        astar.set_maze(maze)
        try:
            leg, resultant_pos = astar.make_path(current_orientation, end_points[i][2])
        except:
            print("Path not found")
            break
        print(leg)
        print(resultant_pos)
        current_start = resultant_pos
        current_orientation = end_points[i][2]
        path.append(leg)
        i += 1

        i = 0
        for leg in path:
            for movement in leg:
                superpath.append(movement)
            superpath.append(end_points[i][3])
            i += 1

    commands = get_commands(superpath)

    return jsonify({
        "data": {'commands': commands},
        "error": None
    })

if __name__ == '__send_commands__':
    app.run(host='0.0.0.0', port=5000, debug=True)


    

    
