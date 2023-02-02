from enum import IntEnum
from obstacle import Obstacle

class CellStatus(IntEnum):
    EMPTY = 0 # empty
    START = 1 # starting area
    BOUNDARY = 2 # boundary area around obstacle
    OBS = 3 # obstacle
    VISITED_OBS = 4 # obstacle visited
    PATH = 5 # 5 and above is path to take

class Cell:
    def __init__(self, x_coordinate, y_coordinate, status: CellStatus = CellStatus.EMPTY):
        # self.direction = None
        self.x_coordinate = x_coordinate
        self.y_coordinate = y_coordinate
        self.status = status
        self.obstacle = None

    def draw_cell(self):
        if self.status == CellStatus.OBS:
            """Draw the obstacle square"""
            pass
        else:
            """Draw normal cell square"""
            pass

    def set_obstacle(self):
        if(self.status == CellStatus.EMPTY):
            self.obstacle = Obstacle(self.x_coordinate, self.y_coordinate)
            self.status = CellStatus.OBS
        elif(self.status == CellStatus.OBS):
            print("This is already an obstacle")

    def remove_obstacle(self):
        self.obstacle = None
        self.status = CellStatus.EMPTY
    
    def set_image(self, count):
        self.obstacle.on_click()
        print(self.obstacle.facing_direction)