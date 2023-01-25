import pygame
import os
from Robot import Robot

WIDTH, HEIGHT = 1024, 720
FPS = 30
BLOCK_SIZE = 32
WIN = pygame.display.set_mode((WIDTH, HEIGHT))

#COLORS
WHITE = (255,255,255)
BLACK = (0, 0, 0)

ROBOT_IMG = pygame.image.load(os.path.join('Assets', 'car.png'))

grid_rects = []
def draw_grid():
    WIN.fill(WHITE)
    for rects in grid_rects:
        pygame.draw.rect(WIN, BLACK, rects)
    return

def create_grid():
    counter = 0
    for i in range(21):
        horizontal = pygame.Rect(0, counter, BLOCK_SIZE*20, 2)
        vertical = pygame.Rect(counter, 0, 2, BLOCK_SIZE*20)
        grid_rects.append(horizontal)
        grid_rects.append(vertical)
        counter += BLOCK_SIZE

def draw_robot(robot):
    WIN.blit(robot.img, (robot.rect.x, robot.rect.y))

def robot_movement_handler(keys_pressed, robot):
    if keys_pressed[pygame.K_w]:
        robot.rect.y -= BLOCK_SIZE
        robot.img = pygame.transform.rotate(ROBOT_IMG, 0)
    if keys_pressed[pygame.K_s]:
        robot.rect.y += BLOCK_SIZE
        robot.img = pygame.transform.rotate(ROBOT_IMG, 180)
    if keys_pressed[pygame.K_a]:
        robot.rect.x -= BLOCK_SIZE
        robot.img = pygame.transform.rotate(ROBOT_IMG, 90)
    if keys_pressed[pygame.K_d]:
        robot.rect.x += BLOCK_SIZE
        robot.img = pygame.transform.rotate(ROBOT_IMG, 270)

def main():
    clock = pygame.time.Clock()
    run = True
    create_grid()
    robot = Robot(0, 0, ROBOT_IMG)
    while run:
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
        draw_grid()

        keys_pressed = pygame.key.get_pressed()
        robot_movement_handler(keys_pressed, robot)
        draw_robot(robot)
        
        pygame.display.update()
    pygame.quit()

if __name__ == "__main__":
    main()
