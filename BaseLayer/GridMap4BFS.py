# input:
# array_allBarrier:n*2 array, each row is the coordinates of each barrier.
# robot: start of the path  target: end of the path
# output:
# path: The specific path is stored in array.
# result: length of path.



import numpy as np



import cppTest


def gridMap4BFS(array_allBarrier, robot, target):


    extendPixel = 3


    barrier_x_min = int(min(np.min(array_allBarrier, axis=0)[0], target.x, robot.x))
    barrier_x_max = int(max(np.max(array_allBarrier, axis=0)[0], target.x, robot.x))
    barrier_y_min = int(min(np.min(array_allBarrier, axis=0)[1], target.y, robot.y))
    barrier_y_max = int(max(np.max(array_allBarrier, axis=0)[1], target.y, robot.y))

    field_x = barrier_x_max - barrier_x_min + 1 + extendPixel * 2
    field_y = barrier_y_max - barrier_y_min + 1 + extendPixel * 2


    map = np.zeros((field_x + 2, field_y + 2))

    for i in range(map.shape[0]):
        map[i, 0] = 1
        map[i, map.shape[1] - 1] = 1
    for i in range(map.shape[1]):
        map[0, i] = 1
        map[map.shape[0] - 1, i] = 1


    for i in range(array_allBarrier.shape[0]):
        x_map = array_allBarrier[i, 0] + (0 - barrier_x_min) + extendPixel + 1
        y_map = array_allBarrier[i, 1] + (0 - barrier_y_min) + extendPixel + 1
        map[x_map, y_map] = 1


    path, result = cppTest.BFS_cpp(map, np.array(
        [robot.x + (0 - barrier_x_min) + extendPixel + 1, robot.y + (0 - barrier_y_min) + extendPixel + 1]),
                           np.array([target.x + (0 - barrier_x_min) + extendPixel + 1,
                                        target.y + (0 - barrier_y_min) + extendPixel + 1]))

    return path, result
