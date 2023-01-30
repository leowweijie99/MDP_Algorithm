import pygame
import os
from Robot import Robot
import constants as const
from simulator import Simulator


def main():
    simulator = Simulator()
    simulator.run()

if __name__ == "__main__":
    main()
