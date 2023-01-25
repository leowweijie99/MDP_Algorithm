import pygame

class Robot:
    def __init__(self, x, y, img):
        self.img = img
        self.width = img.get_width()
        self.height = img.get_height()
        self.rect = pygame.Rect(0, 544, self.width, self.height)