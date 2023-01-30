from enum import IntEnum

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