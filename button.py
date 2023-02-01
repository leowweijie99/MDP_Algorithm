import pygame
import constants as const

class Button():
    def __init__(self, surface: pygame.Surface, pos: tuple, width: int, height: int, color: tuple, text: str, text_color: tuple, function):
        self.surface = surface
        self.x = pos[0]
        self.y = pos[1]
        self.width = width
        self.height = height
        self.color = color
        self.text = text
        self.text_color = text_color
        self.function = function
        self.rect = pygame.Rect(pos[0], pos[1], width, height)

    def draw_button(self):
        pygame.draw.rect(self.surface, self.color, (self.x, self.y, self.width, self.height))
        self.write_text()

    def write_text(self):
        font = pygame.font.SysFont(const.btn_font, const.btn_font_size)
        text = font.render(self.text, False, self.text_color)
        self.surface.blit(text, ((self.x + self.width / 2) - text.get_width() / 2, (self.y + self.height / 2) - text.get_height() / 2))

