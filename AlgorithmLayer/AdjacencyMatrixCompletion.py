# Intalize adjacency matrix
# Edges between nodes in the cluster are set as the maximum, the distance between the virtual node and all nodes is the minimum, and the distance from each node to itself is the minimum.
# The length of all paths starting from the starting node is obtained by BFS, and if there is no feasible solution, BFS returns -1; the edge is set to the maximum value.

import math
import sys
sys.path.append('../BaseLayer/')
import BaseUnits
import GridMap4BFS
import numpy as np
import copy
from multiprocessing import Pool
from functools import partial


def adjacencyMatrixCompletion(robot_aggregation, targetList_aggregation, barrierList_aggregation):


    dimension = 1 + 1 + len(targetList_aggregation)

    D = np.zeros((dimension,dimension))


    for i in range(len(targetList_aggregation)):
        objects = [x for x in targetList_aggregation if (x.parent_robot == targetList_aggregation[i].parent_robot or x.parent_target == targetList_aggregation[i].parent_target)]
        for j in range(len(objects)):
            if targetList_aggregation.index(objects[j]) == i:
                D[i+2,targetList_aggregation.index(objects[j])+2] = np.finfo(float).eps
            else:
                D[i+2, targetList_aggregation.index(objects[j])+2] = 1 / np.finfo(float).eps

    for i in range(dimension):
        D[0,i] = np.finfo(float).eps
        D[i,0] = np.finfo(float).eps

    D[1,1] = np.finfo(float).eps


    D_constant = D.copy()


    pool = Pool(processes = 80)

    for p in range(D.shape[0]):
        for q in range(p, D.shape[1]):
            if D_constant[p,q] == 0:



                partial_callback = partial(callback_fillD, p=p, q=q, D=D)
                pool.apply_async(func=pathLengthCalc,
                                 args=(p, q, robot_aggregation, targetList_aggregation, barrierList_aggregation,),
                                 callback=partial_callback, error_callback=print)

    pool.close()
    pool.join()





    return D,D_constant



def pathLengthCalc(p, q, robot_aggregation, targetList_aggregation, barrierList_aggregation):

    if p == 1:

        array_allBarrier = np.zeros((len(targetList_aggregation) + len(barrierList_aggregation) - 1, 2),
                                    dtype=int)
        index_allBarrier = 0
        for i in range(len(barrierList_aggregation)):
            array_allBarrier[index_allBarrier, 0] = barrierList_aggregation[i].x
            array_allBarrier[index_allBarrier, 1] = barrierList_aggregation[i].y
            index_allBarrier = index_allBarrier + 1
        for i in range(len(targetList_aggregation)):
            if i != (q - 2) and not (
                    targetList_aggregation[i].x == targetList_aggregation[q - 2].x and targetList_aggregation[i].y ==
                    targetList_aggregation[q - 2].y):
                array_allBarrier[index_allBarrier, 0] = targetList_aggregation[i].x
                array_allBarrier[index_allBarrier, 1] = targetList_aggregation[i].y
                index_allBarrier = index_allBarrier + 1

        nonzero_rows = ~np.all(array_allBarrier == 0, axis=1)
        array_allBarrier_nonzero = array_allBarrier[nonzero_rows]


        path, length = GridMap4BFS.gridMap4BFS(array_allBarrier_nonzero, robot_aggregation,
                                               targetList_aggregation[q - 2])

        if length < 0:
            length = 1 / np.finfo(float).eps



        return length



    else:
        
        array_allBarrier = np.zeros((len(targetList_aggregation) + len(barrierList_aggregation) - 2,
                                     2), dtype=int)
        index_allBarrier = 0
        for i in range(len(barrierList_aggregation)):
            array_allBarrier[index_allBarrier, 0] = barrierList_aggregation[i].x
            array_allBarrier[index_allBarrier, 1] = barrierList_aggregation[i].y
            index_allBarrier = index_allBarrier + 1
        for i in range(len(targetList_aggregation)):
            if (i != (q - 2)) and (i != (p - 2) and not ((targetList_aggregation[i].x == targetList_aggregation[
                q - 2].x and targetList_aggregation[i].y == targetList_aggregation[q - 2].y) or (
                                                                 targetList_aggregation[i].x == targetList_aggregation[
                                                             p - 2].x and targetList_aggregation[i].y ==
                                                                 targetList_aggregation[p - 2].y))):
                array_allBarrier[index_allBarrier, 0] = targetList_aggregation[i].x
                array_allBarrier[index_allBarrier, 1] = targetList_aggregation[i].y
                index_allBarrier = index_allBarrier + 1


        nonzero_rows = ~np.all(array_allBarrier == 0, axis=1)
        array_allBarrier_nonzero = array_allBarrier[nonzero_rows]


        _, length = GridMap4BFS.gridMap4BFS(array_allBarrier_nonzero, targetList_aggregation[p - 2],
                                            targetList_aggregation[q - 2])

        if length < 0:
            length = 1 / np.finfo(float).eps

        print("length" + str(p) + "," + str(q) + "finished")

        return length



def callback_fillD(length, p, q, D):
    D[p, q] = length
    D[q, p] = length


