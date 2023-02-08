#COLORS
WHITE = (255,255,255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
LIGHT_NAVY_BLUE = (92, 107, 156)
NAVY_BLUE = (0, 0, 80)
NEON_PINK = (251, 72, 196)
GREY = (192, 192, 192)


#SIMULATOR CONSTANTS
WIDTH, HEIGHT = 1024, 720
FPS = 30
BLOCK_SIZE = 24
MARGIN = 2
MARGIN_BLOCK_SIZE = BLOCK_SIZE + MARGIN
VERTICAL_OFFSET = 50
NUM_OF_BLOCKS = 20
GRID_SIZE = (BLOCK_SIZE+MARGIN)*NUM_OF_BLOCKS
HORIZONTAL_OFFSET = (WIDTH/2) - (GRID_SIZE/2)

#CONTROLS CONSTANTS
btn_font_size = 14
btn_font = "Comic Sans MS"

#ROBOT CONSTANTS
ROBOT_WIDTH = MARGIN_BLOCK_SIZE * 3 - MARGIN
ROBOT_HEIGHT = MARGIN_BLOCK_SIZE * 3 - MARGIN
ROBOT_STARTLOC = [0, 0]
ROBOT_IMG_PATH = 'Assets/car.png'

# draw at x, 19-y