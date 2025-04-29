# Inputs are the key elements in the original view, and outputs are the key elements in the aggregate view.
import sys
sys.path.append('../BaseLayer/')
import MapInit
import BaseUnits
import numpy as np


def fieldConversion(robotList,targetList,barrierList):

    robot_aggregation = BaseUnits.robot(0,0,-1)

    barrierList_aggregation = []
    targetList_aggregation = []


    for i in range(len(robotList)):
        for j in range(len(barrierList)):
            tempbarrier = BaseUnits.barrier(barrierList[j].x - robotList[i].x + 0, barrierList[j].y - robotList[i].y + 0, i)
            barrierList_aggregation.append(tempbarrier)


    for i in range(len(robotList)):
        for j in range(len(targetList)):
            if robotList[i].type == targetList[j].type:
                temptarget = BaseUnits.target(targetList[j].x - robotList[i].x + 0, targetList[j].y - robotList[i].y + 0, -1 ,i ,j)
                targetList_aggregation.append(temptarget)
            else:
                tempbarrier = BaseUnits.barrier(targetList[j].x - robotList[i].x + 0,
                                                targetList[j].y - robotList[i].y + 0, i)
                barrierList_aggregation.append(tempbarrier)


    return robot_aggregation, targetList_aggregation, barrierList_aggregation

