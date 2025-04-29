# All three inputs are n*3 arrays, where the first two columns are the coordinates of the elements and the third column indicates the category to which each element belongs.
# Outputs a list of objects that store information about each element, which is later used to create aggregate view.
# When there are no barriers in  initial situation, the default value of array_barrier is -1.

import numpy as np


import BaseUnits



def mapInit(array_robot, array_target, array_barrier = -1):


    robotList = []
    targetList = []
    barrierList = []


    robot_rows,_ = array_robot.shape
    target_rows,_ = array_target.shape

    if type(array_barrier) is not np.ndarray:
        barrier_rows = 0
    else:
        barrier_rows,_ = array_barrier.shape

    for i in range(robot_rows):
        temprobot = BaseUnits.robot(array_robot[i,0],array_robot[i,1],array_robot[i,2])
        robotList.append(temprobot)

    # 原视图中不存在映射来源，因此来源元素设为-1
    for i in range(target_rows):
        temptarget = BaseUnits.target(array_target[i,0],array_target[i,1],array_target[i,2],-1,-1)
        targetList.append(temptarget)

    for i in range(barrier_rows):
        tempbarrier = BaseUnits.barrier(array_barrier[i,0],array_barrier[i,1],-1)
        barrierList.append(tempbarrier)


    return robotList,targetList,barrierList

