from enum import IntEnum
from obstacle import Obstacle
from goal import Goal
from barrier import Barrier
import pygame
import constants as const

class CellStatus(IntEnum):
    EMPTY = 0 # empty
    START = 1 # starting area
    BOUNDARY = 2 # boundary area around obstacle
    OBS = 3 # obstacle
    VISITED_OBS = 4 # obstacle visited
    PATH = 5 # 5 and above is path to take
    GOAL = 6
    BARRIER = 7

class Cell:
    def __init__(self, x_coordinate, y_coordinate, status: CellStatus = CellStatus.EMPTY):
        # self.direction = None
        self.x_coordinate = x_coordinate
        self.y_coordinate = y_coordinate
        self.status = status
        self.obstacle = None
        self.goal = None

    def set_obstacle(self):
        if(self.status == CellStatus.EMPTY):
            self.obstacle = Obstacle(self.x_coordinate, self.y_coordinate)
            self.status = CellStatus.OBS

    def remove_obstacle(self):
        self.obstacle = None
        self.status = CellStatus.EMPTY

    
    def set_image(self, count):
        self.obstacle.on_click()
        return self.obstacle.facing_direction
    
    def set_goal(self):
        if(self.status == CellStatus.EMPTY):
            self.goal = Goal(self.x_coordinate, self.y_coordinate)
            self.status = CellStatus.GOAL

    def remove_goal(self):
        self.goal = None
        self.status = CellStatus.EMPTY

    def set_barrier(self):
        if(self.status == CellStatus.EMPTY):
            self.barrier = Barrier(self.x_coordinate, self.y_coordinate)
            self.status = CellStatus.BARRIER
    
    def set_normal(self):
        self.status = CellStatus.EMPTY