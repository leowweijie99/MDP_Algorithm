from Astar import Astar
from Robot import RobotMoves

def get_commands(movement_list: list) -> list:
    
    command_list = []

    for movement in movement_list:
        if movement == RobotMoves.FORWARD:
            command_list.append("FW10")
        elif movement == RobotMoves.BACKWARD:
            command_list.append("BW10")
        elif movement == RobotMoves.FORWARD_LEFT:
            command_list.append("FL30")
        elif movement == RobotMoves.FORWARD_RIGHT:
            command_list.append("FR00")
        elif movement == RobotMoves.BACKWARD_LEFT:
            command_list.append("BL00")
        elif movement == RobotMoves.BACKWARD_RIGHT:
            command_list.append("BR00")
        elif type(movement) is list:
            command_list.append("SNAP{}".format(movement[0]))

    command_list.append("FIN")

    compressed_commands = [command_list[0]]
    for i in range(1, len(command_list)):
        if command_list[i].startswith("BW") and compressed_commands[-1].startswith("BW"):
            steps = int(compressed_commands[-1][2:])
            if steps != 90:
                compressed_commands[-1] = "BW{}".format(steps + 10)
                continue

        elif command_list[i].startswith("FW") and compressed_commands[-1].startswith("FW"):
            steps = int(compressed_commands[-1][2:])
            if steps != 90:
                compressed_commands[-1] = "FW{}".format(steps + 10)
                continue

        compressed_commands.append(command_list[i])

    return compressed_commands

'''movements = [RobotMoves.FORWARD, RobotMoves.FORWARD, RobotMoves.FORWARD, RobotMoves.FORWARD, RobotMoves.BACKWARD, RobotMoves.BACKWARD,
             RobotMoves.FORWARD_LEFT, [0]]

commands = get_commands(movements)

print(commands)'''