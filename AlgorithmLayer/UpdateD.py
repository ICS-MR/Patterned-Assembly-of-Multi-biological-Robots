# Remove obstacles that no longer exist in the aggregate view based on the target points that have passed so far, and populate the adjacency matrix.

import sys
sys.path.append('../BaseLayer/')
import BaseUnits
import GridMap4BFS
import numpy as np
import copy
from multiprocessing import Pool
from functools import partial



def updateD(curTabu, curPath, curD, targetList_aggregation, barrierList_aggregation):


    if len(curTabu) == 2:
        return 0

    else:


        robot_finished = []

        target_finished = []
        for i in range(2,len(curPath)):

            if curPath[i] != 0:
                robot_finished.append(targetList_aggregation[curPath[i] - 2].parent_robot)
                target_finished.append(targetList_aggregation[curPath[i] - 2].parent_target)
            else:
                break
                

        n = curD.shape[0]
        allElements = list(range(n))
        J = [x for x in allElements if x not in curTabu]


        for i in range(len(J)):



            targetList_remain = [obj for idx, obj in enumerate(targetList_aggregation) if
                                 obj.parent_robot not in robot_finished and idx not in [curTabu[-1] - 2,
                                                                                        J[i] - 2]]
            barrierList_remain = [obj for obj in barrierList_aggregation if obj.parent_robot not in robot_finished]

            array_allBarrier = np.zeros((len(targetList_remain) + len(barrierList_remain),
                                         2), dtype=int)
            index_allBarrier = 0
            for j in range(len(barrierList_remain)):
                array_allBarrier[index_allBarrier, 0] = barrierList_remain[j].x
                array_allBarrier[index_allBarrier, 1] = barrierList_remain[j].y
                index_allBarrier = index_allBarrier + 1

            for j in range(len(targetList_remain)):

                if not ((targetList_remain[j].x == targetList_aggregation[J[i] - 2].x and
                         targetList_remain[j].y == targetList_aggregation[J[i] - 2].y) and (targetList_remain[j].parent_target not in target_finished)):
                    array_allBarrier[index_allBarrier, 0] = targetList_remain[j].x
                    array_allBarrier[index_allBarrier, 1] = targetList_remain[j].y
                    index_allBarrier = index_allBarrier + 1

            nonzero_rows = ~np.all(array_allBarrier == 0, axis=1)
            array_allBarrier_nonzero = array_allBarrier[nonzero_rows]


            _, length = GridMap4BFS.gridMap4BFS(array_allBarrier_nonzero, targetList_aggregation[curTabu[-1] - 2],
                                                targetList_aggregation[J[i] - 2])

            if length < 0:
                length = 1 / np.finfo(float).eps

            curD[curTabu[-1], J[i]] = length



        return 0
