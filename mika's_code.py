import rospy
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid


def print_map_to_file():
    with open("grid.txt", "w+") as grid_file:
        global grid
        for row in reversed(grid):
            for cell in row:
                grid_file.write("1") if cell else grid_file.write("0")
            grid_file.write("\n")


def creat_occupancy_grid(my_map):
    # creating the occupancy grid
    global grid
    grid = [[None] * my_map.info.width for i in xrange(my_map.info.height)]
    for i in xrange(my_map.info.height):
        for j in xrange(my_map.info.width):
            if my_map.data[i * my_map.info.width + j] == 0:
                grid[i][j] = False
            else:
                grid[i][j] = True


if __name__ == "__main__":
    # rospy.init_node('add_two_ints_server')
    rospy.wait_for_service('static_map')
    try:
        get_static_map = rospy.ServiceProxy('static_map', GetMap)
        response = get_static_map()
        rospy.loginfo("Received a %d X %d map @ %.3f m/px" % (
            response.map.info.width, response.map.info.height, response.map.info.resolution))
        creat_occupancy_grid(response.map)

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)

    print_map_to_file()
