import pygame
import os
from robot import Robot
import constants as const
from simulator import Simulator


def main():
    simulator = Simulator()
    simulator.run()

if __name__ == "__main__":
    main()
