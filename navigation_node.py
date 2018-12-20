#!/usr/bin/python

import math
import sys

import numpy as np
import rospy
from nav_msgs.srv import GetMap

import a_star


def main():
    rospy.init_node("navigation_node", argv=sys.argv)

    starting_location, goal_location, robot_size = get_params()

    try:
        get_static_map = rospy.ServiceProxy('static_map', GetMap)
        response = get_static_map()
        rospy.loginfo("Received a %d X %d map @ %.3f m/px" % (
            response.map.info.width, response.map.info.height, response.map.info.resolution))

        # Create new grid by robot size and update the starting and goal location
        grid = create_occupancy_grid(response.map, robot_size)
        starting_location = update_grid_location(response, robot_size, starting_location)
        goal_location = update_grid_location(response, robot_size, goal_location)

        # use flipud for A*
        flipud_grid = np.flipud(grid)
        # active A* algorithm
        t, r_path, r_symbols = a_star.PathPlanner(np.array(flipud_grid), False).a_star(np.array(starting_location),
                                                                                       np.array(goal_location))

        # TODO
        # Write the grid to 'new_grid' file
        print_map_to_file(grid)
        # Write the path to 'path' file
        print_path_string_to_file(starting_location, goal_location, r_path)

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)


def get_params():
    starting_location = 0, 0
    goal_location = -80, -30
    robot_size = 0.35

    if rospy.has_param('/starting_location'):
        starting_location = tuple(map(float, rospy.get_param('/starting_location').split(',')))
    if rospy.has_param('/goal_location'):
        goal_location = tuple(map(float, rospy.get_param('/goal_location').split(',')))
    if rospy.has_param('/robot_size'):
        robot_size = rospy.get_param('/robot_size')
    rospy.wait_for_service('static_map')

    return starting_location, goal_location, robot_size


def print_map_to_file(grid):
    with open("new_grid.txt", "w+") as grid_file:
        # Use converted_grid
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


def update_grid_location(response, robot_size, point):
    x, y = point
    new_x = int(math.ceil(abs((response.map.info.origin.position.x - x) / robot_size))) - 1
    new_y = int(math.ceil(abs(((response.map.info.origin.position.y - y) / robot_size)))) - 1
    return [new_x, new_y]


def create_occupancy_grid(my_map, robot_size):
    # cells for robot
    cell = int(math.ceil(robot_size / my_map.info.resolution))

    width = int(math.floor(my_map.info.width / cell))
    height = int(math.floor(my_map.info.height / cell))

    # build grid
    grid = [[None] * width for _ in xrange(height)]

    # go over the old grid and check for every old cell if there is an obstacle in the smaller cells
    old_height = 0
    old_width = 0
    for i in xrange(height):
        for j in xrange(width):
            obstacle = False
            for k in xrange(old_height, old_height + cell):
                if not obstacle:
                    for z in xrange(old_width, old_width + cell):
                        if my_map.data[k * my_map.info.width + z] != 0:
                            grid[i][j] = True
                            obstacle = True
                            break
            if not obstacle:
                grid[i][j] = False
            old_width += cell
        old_width = 0
        old_height += cell

    return grid


if __name__ == "__main__":
    main()
