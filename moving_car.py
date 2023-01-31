import constants
import pygame

carImg = pygame.image.load('Assets/car.png')
gameDisplay = pygame.display.set_mode((constants.WIDTH, constants.HEIGHT))
clock = pygame.time.Clock()


def car(x, y):
    gameDisplay.blit(carImg, (x, y))

x = (constants.WIDTH * 0.45)
y = (constants.HEIGHT * 0.8)

x_change = 0
car_speed = 0

crashed  = False

while not crashed:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            crashed = True

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                x_change = -0.5
            elif event.key == pygame.K_RIGHT:
                x_change = 0.5
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
                    x_change = 0
            

    x += x_change

    gameDisplay.fill(constants.WHITE)
    car(x, y)

    pygame.display.update()
    clock.tick(60)

pygame.quit()
quit()