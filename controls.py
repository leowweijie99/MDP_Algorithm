# Panel for all the buttons
from button import *
import Astar

class Controls:

    def __init__(self, surface, simulator):
        self.surface = surface
        self.simulator = simulator
        self.robot = self.simulator.robot
        self.buttons = []

        # ------------- Initialize all the buttons here --------------------------------------------------
        # Obstacle buttons
        btnAddObstacle = Button(self.surface, (800, 50), 120, 25, const.WHITE, "Add Obstacle", const.BLACK, "ADD")
        self.buttons.append(btnAddObstacle)
        btnRemoveObstacle = Button(self.surface, (800, 90), 120, 25, const.WHITE, "START", const.BLACK, "START")
        self.buttons.append(btnRemoveObstacle)

        # Movement buttons
        btnForward = Button(self.surface, (800, 150), 120, 25, const.WHITE, "Forward", const.BLACK, "FORWARD")
        self.buttons.append(btnForward)
        btnForwardLeft = Button(self.surface, (800, 180), 120, 25, const.WHITE, "Forward Left", const.BLACK, "FORWARD_LEFT")
        self.buttons.append(btnForwardLeft)
        btnForwardRight = Button(self.surface, (800, 210), 120, 25, const.WHITE, "Forward Right", const.BLACK, "FORWARD_RIGHT")
        self.buttons.append(btnForwardRight)
        
        btnBackwards = Button(self.surface, (800, 280), 120, 25, const.WHITE, "Backwards", const.BLACK, "BACKWARD")
        self.buttons.append(btnBackwards)
        btnBackwardLeft = Button(self.surface, (800, 310), 120, 25, const.WHITE, "Backward Left", const.BLACK, "BACKWARD_LEFT")
        self.buttons.append(btnBackwardLeft)
        btnBackwardRight = Button(self.surface, (800, 340), 120, 25, const.WHITE, "Backward Right", const.BLACK, "BACKWARD_RIGHT")
        self.buttons.append(btnBackwardRight)


    def draw_buttons(self):
        for btn in self.buttons:
            btn.draw_button()

    def click_selected_button(self, mousePos: tuple):
        for btn in self.buttons:
            if ((btn.x < mousePos[0] < btn.x + btn.width) and (btn.y < mousePos[1] < btn.y + btn.height)): # Check if mosPos is within a button's x and y coord
                func = btn.function
                if func == "ADD":
                    self.generate_test_obstacles()
                    return True
                elif func == "START":
                    self.simulator.on_start()
                        
                    return True
                elif func == "FORWARD":
                    self.robot.move_forward(10)
                    return True
                elif func == "FORWARD_LEFT":
                    self.robot.move_forward_left()
                    return True
                elif func == "FORWARD_RIGHT":
                    self.robot.move_forward_right()
                    return True
                elif func == "BACKWARD":
                    self.robot.move_backward(10)
                    return True
                elif func == "BACKWARD_LEFT":
                    self.robot.move_backward_left()
                    return True
                elif func == "BACKWARD_RIGHT":
                    self.robot.move_backward_right()
                    return True 
        #print(self.robot.location)
        return False
    def generate_test_obstacles(self):
        self.simulator.grid.set_cell_as_obstacle(4, 6)
        #self.simulator.grid.set_cell_image_direction(4, 6, 1)

        self.simulator.grid.set_cell_as_obstacle(0, 15)
        #self.simulator.grid.set_cell_image_direction(0, 15, 2)

        self.simulator.grid.set_cell_as_obstacle(3, 19)
        #self.simulator.grid.set_cell_image_direction(3, 19, 2)

        self.simulator.grid.set_cell_as_obstacle(9, 14)
        #self.simulator.grid.set_cell_image_direction(9, 14, 0)

        self.simulator.grid.set_cell_as_obstacle(13, 12)
        #self.simulator.grid.set_cell_image_direction(13, 12, 2)

        self.simulator.grid.set_cell_as_obstacle(13, 13)
        #self.simulator.grid.set_cell_image_direction(13, 13, 0)

        self.simulator.grid.set_cell_as_obstacle(19, 19)
        #self.simulator.grid.set_cell_image_direction(19, 19, 3)

        self.simulator.grid.set_cell_as_obstacle(18, 0)
        self.simulator.grid.set_cell_image_direction(18, 0, 0)