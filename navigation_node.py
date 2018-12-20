#!/usr/bin/python

import rospy, sys
import math
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import a_star as a


def print_map_to_file():
    with open("new_grid.txt", "w+") as grid_file:
        # Use converted_grid
        global grid
        for row in reversed(grid):
            for cell in row:
                grid_file.write("1") if cell else grid_file.write("0")
            grid_file.write("\n")


def print_path_string_to_file(start_point, goal_point, path):
    """
    Created path file.
    :param start_point: starting point, marked as 'S'.
    :param goal_point: goal point, marked as 'G'.
    :param path: correct path.
    :return: null
    """
    # Write to file
    with open("path.txt", "w+") as grid_file:
        grid_file.write("Start point: " + " ".join(str(x) for x in start_point) + "\n")
        for line in path:
            grid_file.write(" ".join(str(x) for x in line) + "\n")
        grid_file.write("Goal point: " + " ".join(str(x) for x in goal_point))


def print_path_to_file(start_point, goal_point, path, symbols):
    """
    Created path file.
    :param start_point: starting point, marked as 'S'.
    :param goal_point: goal point, marked as 'G'.
    :param path: correct path.
    :param symbols: Symbols that marks the path direction.
    :return: null
    """
    lines = []

    global grid
    # Create string lines for each line in converted grid
    for row in reversed(grid):
        line = ''
        for cell in row:
            if cell:
                line += '1'
            else:
                line += '0'
        line += '\n'
        lines.append(line)
    # Update start point cell to be 'S'
    x_start_point, y_start_point = start_point
    line = list(lines[y_start_point])
    line[x_start_point] = 'S'
    lines[y_start_point] = "".join(line)

    # Update goal point cell to be 'G'
    x_goal_point, y_goal_point = goal_point
    line = list(lines[y_goal_point])
    line[x_goal_point] = 'G'
    lines[y_goal_point] = "".join(line)
    # Remove first symbol(we have one extra symbol)
    symbols.pop(0)
    # For each move, put on map it's relative symbol
    for move, symbol in zip(path, symbols):
        x_current, y_current = move
        line = list(lines[y_current])
        line[x_current] = symbol
        lines[y_current] = "".join(line)
    # Write to file
    with open("path.txt", "w+") as grid_file:
        for line in lines:
            grid_file.write(line)


def convert_old_grid_point(size, point):
    """
    Calculates new point in new grid.
    :param size: new robot size
    :param point: point to convert
    :return: converted point
    """
    global grid
    x, y = point
    number_of_rows = len(grid)
    # Assume that origin point is left down place
    new_point_x = int(math.ceil(abs((response.map.info.origin.position.x - x) / size))) - 1
    new_point_y = int(math.ceil(abs(((response.map.info.origin.position.y - y) / size)))) - 1

    return [new_point_x, new_point_y]


def create_grid_by_robot_size(my_map, size):
    """
    Creates new grid with perspective to new robot size
    :param my_map: map to use it's info
    :param size: new robot size
    :return: null
    """
    global grid
    # Calc how much cells the robot needs
    new_resolution = int(math.ceil(size / my_map.info.resolution))
    # Calc new width and height with perspective to new resolution
    new_width = int(math.floor(my_map.info.width / new_resolution))
    new_height = int(math.floor(my_map.info.height / new_resolution))
    grid = [[None] * new_width for i in xrange(new_height)]

    i_old_size = 0
    j_old_size = 0
    for i in xrange(new_height):
        for j in xrange(new_width):
            # Mark by default that there is not obstacle
            found_obs = False
            for x in range(i_old_size, i_old_size + new_resolution):
                if found_obs:
                    break
                for y in range(j_old_size, j_old_size + new_resolution):
                    # Check if there is obstacle
                    if my_map.data[x * my_map.info.width + y] != 0:
                        grid[i][j] = True
                        # Stop the iteration as we already mark this cell
                        found_obs = True
                        break
            # Check if there wasn't any obstacle and mark this cell to False
            if not found_obs:
                grid[i][j] = False
            j_old_size += new_resolution
        j_old_size = 0
        i_old_size += new_resolution


def find_closest_empty_place_vertical(start_point, g, direction):
    """
    Find closest empty place vertical
    :param start_point: point to start looking for
    :param g: grid
    :param direction: direction to move
    :return: tuple of new goal point and how it's close to old point
    """
    global grid
    try:
        x, y = start_point
        counter = 0
        while g[y][x]:
            x += direction
            counter += 1
        return [x, y], counter
    except e:
        # If there will be exception(out of borders) - return origin point and max int(so it will
        # be chosen as best point)
        return start_point, sys.maxint


def find_closest_empty_place_horizontal(start_point, g, direction):
    """
    Find closest empty place horizontal
    :param start_point: point to start looking for
    :param g: grid
    :param direction: direction to move
    :return: tuple of new goal point and how it's close to old point
    """
    try:
        x, y = start_point
        counter = 0
        while g[y][x]:
            y += direction
            counter += 1
        return [x, y], counter
    except e:
        # If there will be exception(out of borders) - return origin point and max int(so it will
        # be chosen as best point)
        return start_point, sys.maxint


def validate_goal_location(g_location, g):
    """
    Validate that goal location is not on obstacle. If it is, find closest place to it
    :param g_location: goal location
    :param g: grid
    :return: Goal location not on an obstacle
    """
    x, y = g_location
    # Check if on obstacle
    if g[y][x]:
        rospy.loginfo("Goal point is on an obstacle!!!!, changing goal point...")
        # Check for all direction and find closest point to mark as goal
        new_g_right, c_right = find_closest_empty_place_horizontal(g_location, g, 1)
        new_g_left, c_left = find_closest_empty_place_horizontal(g_location, g, -1)

        new_g_down, c_down = find_closest_empty_place_vertical(g_location, g, 1)
        new_g_up, c_up = find_closest_empty_place_vertical(g_location, g, -1)
        # Get smallest delta
        smallest_delta = min(c_right, c_left, c_down, c_up)
        # Return corresponding goal position
        if smallest_delta == c_right:
            return new_g_right
        elif smallest_delta == c_left:
            return new_g_left
        elif smallest_delta == c_up:
            return new_g_up
        elif smallest_delta == c_down:
            return new_g_down
    else:
        return g_location


if __name__ == "__main__":
    rospy.init_node("navigation_node", argv=sys.argv)
    # Default parameters
    starting_location = 0, 0
    goal_location = -80, -30
    robot_size = 0.35
    # Get parameter from launch file
    if rospy.has_param('/starting_location'):
        starting_location = tuple(map(float, rospy.get_param('/starting_location').split(',')))
    if rospy.has_param('/goal_location'):
        goal_location = tuple(map(float, rospy.get_param('/goal_location').split(',')))
    if rospy.has_param('/robot_size'):
        robot_size = rospy.get_param('/robot_size')

    rospy.wait_for_service('static_map')
    try:
        get_static_map = rospy.ServiceProxy('static_map', GetMap)
        response = get_static_map()
        rospy.loginfo("Received a %d X %d map @ %.3f m/px" % (
            response.map.info.width, response.map.info.height, response.map.info.resolution))
        # creat_occupancy_grid(response.map)
        # Create new grid by robot size
        create_grid_by_robot_size(response.map, robot_size)
        # Convert start and goal point to be with perspective to new grid map
        converted_start_point = convert_old_grid_point(robot_size, starting_location)
        converted_goal_point = convert_old_grid_point(robot_size, goal_location)
        # Flip grid
        fliped_grid = np.flipud(grid)
        # Check if goal point is on obstacle, and update it if necessary
        converted_goal_point = validate_goal_location(converted_goal_point, fliped_grid)
        # Use the A_start algorithm - it's require the given map to be flip and as numpy array
        test_planner = a.PathPlanner(np.array(fliped_grid), False)
        # Run the algorithm.
        t, r_path, r_symbols = test_planner.a_star(np.array(converted_start_point), np.array(converted_goal_point))
        # Write the grid to 'new_grid' file
        print_map_to_file()
        # Write the path to 'path' file
        # print_path_to_file(converted_start_point, converted_goal_point, r_path, r_symbols)
        print_path_string_to_file(converted_start_point, converted_goal_point, r_path)
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)
