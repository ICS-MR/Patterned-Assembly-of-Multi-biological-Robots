# Visualize data


import os

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Circle
import numpy as np

import GridMap4BFS

from matplotlib.animation import  ArtistAnimation

import subprocess

# Need to change to your local ffmpeg path!.
ffmpegPath = 'D:\\ffmpeg\\bin\\ffmpeg.exe'



def originalViewPlot(robot_original, target_original, barrier_original, min_xlim,max_xlim, min_ylim, max_ylim, flag_save):

    fig = plt.figure(figsize=(6,6))
    plt.xlim(min_xlim, max_xlim)
    plt.ylim(min_ylim, max_ylim)

    for i in range(len(target_original)):
        test = plt.Circle((target_original[i].x + 0.5, target_original[i].y + 0.5), 0.5, facecolor='red', edgecolor='red', alpha=1)
        fig.gca().add_patch(test)

    for i in range(len(barrier_original)):
        test = plt.Circle((barrier_original[i].x + 0.5, barrier_original[i].y + 0.5), 0.5, facecolor='black', edgecolor='black', alpha=1)
        fig.gca().add_patch(test)

    for i in range(len(robot_original)):
        test = plt.Circle((robot_original[i].x + 0.5, robot_original[i].y + 0.5), 0.5, facecolor='green', edgecolor='green', alpha=1)
        fig.gca().add_patch(test)

    if flag_save:
        plt.savefig('Results/OriginalView.pdf')
    plt.show()




def aggregationViewPlot(target_aggregation,barrier_aggregation, min_xlim,max_xlim, min_ylim, max_ylim, flag_save):

    fig = plt.figure(figsize=(6,6))
    plt.xlim(min_xlim, max_xlim)
    plt.ylim(min_ylim, max_ylim)


    for i in range(len(target_aggregation)):
        test = plt.Circle((target_aggregation[i].x + 0.5, target_aggregation[i].y + 0.5), 0.5, facecolor='red', edgecolor='red', alpha=1)
        fig.gca().add_patch(test)

    for i in range(len(barrier_aggregation)):
        test = plt.Circle((barrier_aggregation[i].x + 0.5, barrier_aggregation[i].y + 0.5), 0.5, facecolor='black', edgecolor='black', alpha=1)
        fig.gca().add_patch(test)

    test = plt.Circle((0 + 0.5, 0 + 0.5), 0.5, facecolor='green', edgecolor='green', alpha=1)
    fig.gca().add_patch(test)

    if flag_save:
        plt.savefig('Results/AggregationView.pdf')
    plt.show()


def iterativeResultPlot(L_ave, L_best, flag_save):
    NC = np.linspace(0, len(L_ave), len(L_ave))
    plt.plot(NC, L_best, color='blue')
    plt.plot(NC, L_ave, color='red')
    if flag_save:
        plt.savefig('Results/IterativeResult.pdf')
    plt.show()

# Get the specific motion steps of the path.
def detailPathCalc(R_best, robot_aggregation,  target_aggregation, barrier_aggregation):

    bestPath = R_best[-1].copy().astype(int)
    bestPath = bestPath[bestPath != 0]


    # for i in range(1, len(bestPath)):
    #     print(target_aggregation[bestPath[i] - 2].x, target_aggregation[bestPath[i] - 2].y)


    robot_finished = []
    totalLength = 0
    path = []
    for i in range(len(bestPath) - 1):

        if i == 0:
            targetList_remain = [obj for idx, obj in enumerate(target_aggregation) if idx not in [bestPath[i + 1] - 2]]
            barrierList_remain = barrier_aggregation

            array_allBarrier = np.zeros((len(targetList_remain) + len(barrierList_remain),
                                         2), dtype=int)
            index_allBarrier = 0
            for j in range(len(barrierList_remain)):
                array_allBarrier[index_allBarrier, 0] = barrierList_remain[j].x
                array_allBarrier[index_allBarrier, 1] = barrierList_remain[j].y
                index_allBarrier = index_allBarrier + 1

            for j in range(len(targetList_remain)):

                if not (targetList_remain[j].x == target_aggregation[bestPath[i + 1] - 2].x and targetList_remain[j].y == target_aggregation[bestPath[i + 1] - 2].y):
                    array_allBarrier[index_allBarrier, 0] = targetList_remain[j].x
                    array_allBarrier[index_allBarrier, 1] = targetList_remain[j].y
                    index_allBarrier = index_allBarrier + 1

            nonzero_rows = ~np.all(array_allBarrier == 0, axis=1)
            array_allBarrier_nonzero = array_allBarrier[nonzero_rows]

            p, length = GridMap4BFS.gridMap4BFS(array_allBarrier_nonzero, robot_aggregation, target_aggregation[bestPath[i + 1] - 2])

            # print(length)

            totalLength = totalLength + length



            p.reverse()
            x_items = p[::2]
            y_items = p[1::2]
            for k in range(len(x_items) - 1):
                path.append([x_items[k + 1] - x_items[k], y_items[k + 1] - y_items[k]])

            robot_finished.append(target_aggregation[bestPath[i + 1] - 2].parent_robot)

            robot_finished.append(target_aggregation[bestPath[i + 1] - 2].parent_robot)

        else:
            targetList_remain = [obj for idx, obj in enumerate(target_aggregation) if
                                 obj.parent_robot not in robot_finished and idx not in [bestPath[i] - 2,
                                                                                        bestPath[i + 1] - 2]]
            barrierList_remain = [obj for obj in barrier_aggregation if obj.parent_robot not in robot_finished]

            array_allBarrier = np.zeros((len(targetList_remain) + len(barrierList_remain),
                                         2), dtype=int)
            index_allBarrier = 0
            for j in range(len(barrierList_remain)):
                array_allBarrier[index_allBarrier, 0] = barrierList_remain[j].x
                array_allBarrier[index_allBarrier, 1] = barrierList_remain[j].y
                index_allBarrier = index_allBarrier + 1

            for j in range(len(targetList_remain)):
                if not (targetList_remain[j].x == target_aggregation[bestPath[i + 1] - 2].x and targetList_remain[j].y == target_aggregation[bestPath[i + 1] - 2].y) or (targetList_remain[j].x == target_aggregation[bestPath[i] - 2].x and targetList_remain[j].y == target_aggregation[bestPath[i] - 2].y):
                    array_allBarrier[index_allBarrier, 0] = targetList_remain[j].x
                    array_allBarrier[index_allBarrier, 1] = targetList_remain[j].y
                    index_allBarrier = index_allBarrier + 1

            nonzero_rows = ~np.all(array_allBarrier == 0, axis=1)
            array_allBarrier_nonzero = array_allBarrier[nonzero_rows]

            p, length = GridMap4BFS.gridMap4BFS(array_allBarrier_nonzero, target_aggregation[bestPath[i] - 2], target_aggregation[bestPath[i + 1] - 2])

            totalLength = totalLength + length

            # print(length)


            p.reverse()
            x_items = p[::2]
            y_items = p[1::2]
            for k in range(len(x_items) - 1):
                path.append([x_items[k + 1] - x_items[k], y_items[k + 1] - y_items[k]])

            robot_finished.append(target_aggregation[bestPath[i + 1] - 2].parent_robot)

    return path,totalLength

# Use a video to display how the movement appears in the aggregate view.
def processInAggregationView(target_aggregation, barrier_aggregation, path, min_xlim,max_xlim, min_ylim, max_ylim, fig, flag_save):
    fig.clf()


    plt.xlim(min_xlim, max_xlim)
    plt.ylim(min_ylim, max_ylim)
    plt.ion()

    robot = [0,0]
    robotPath = [[robot[0], robot[1]]]

    sequenceIndex = 0

    for i in range(len(target_aggregation)):
        test = plt.Circle((target_aggregation[i].x + 0.5, target_aggregation[i].y + 0.5), 0.5, facecolor='red', edgecolor='red', alpha=1)
        fig.gca().add_patch(test)

    for i in range(len(barrier_aggregation)):
        test = plt.Circle((barrier_aggregation[i].x + 0.5, barrier_aggregation[i].y + 0.5), 0.5, facecolor='black', edgecolor='black', alpha=1)
        fig.gca().add_patch(test)

    test = plt.Circle((robot[0] + 0.5, robot[1] + 0.5), 0.5, facecolor='green', edgecolor='green', alpha=1)
    fig.gca().add_patch(test)

    if flag_save:
        filename = 'Results/Sequence_Aggregation/' + str(sequenceIndex) + '.png'
        plt.savefig(filename)

    ta_exclude = set()
    ba_exclude = set()

    for p in range(len(path)):

        plt.pause(0.2)

        fig.clf()

        plt.xlim(min_xlim, max_xlim)
        plt.ylim(min_ylim, max_ylim)

        robot[0] = robot[0] + path[p][0]
        robot[1] = robot[1] + path[p][1]

        robotPath.append([robot[0], robot[1]])



        for obj in target_aggregation:
            if obj.x == robot[0] and obj.y == robot[1]:
                ta_exclude.add(obj)

                curParent = obj.parent_robot

                for obj in target_aggregation:
                    if obj in ta_exclude:
                        continue
                    if obj.parent_robot == curParent:
                        ta_exclude.add(obj)

                for obj in barrier_aggregation:
                    if obj.parent_robot == curParent:
                        ba_exclude.add(obj)

        curTa_a = [obj for obj in target_aggregation if obj not in ta_exclude]
        curBa_a = [obj for obj in barrier_aggregation if obj not in ba_exclude]





        for i in range(len(curTa_a)):
            test = plt.Circle((curTa_a[i].x + 0.5, curTa_a[i].y + 0.5), 0.5, facecolor='red', edgecolor='red', alpha=1)
            fig.gca().add_patch(test)

        for i in range(len(curBa_a))  :

            test = plt.Circle((curBa_a[i].x+0.5,curBa_a[i].y+0.5),0.5,facecolor='black',edgecolor='black',alpha=1)
            fig.gca().add_patch(test)


        for j in range(len(robotPath)):
            if j != len(robotPath) - 1:
                test = plt.Circle((robotPath[j][0] + 0.5, robotPath[j][1] + 0.5), 0.5, facecolor='green',
                                  edgecolor='green', alpha=0.2)
                fig.gca().add_patch(test)
            else:
                test = plt.Circle((robotPath[j][0] + 0.5, robotPath[j][1] + 0.5), 0.5, facecolor='green',
                                  edgecolor='green', alpha=1)
                fig.gca().add_patch(test)

        if flag_save:
            sequenceIndex = sequenceIndex + 1
            filename = 'Results/Sequence_Aggregation/' + str(sequenceIndex) + '.png'
            plt.savefig(filename)



    plt.pause(0.2)


    if flag_save:
        os.system(f'{ffmpegPath} -framerate 10 -i Results/Sequence_Aggregation/%d.png Results/aggregation.mp4')

        for filename in os.listdir('Results/Sequence_Aggregation/'):
            file_path = os.path.join('Results/Sequence_Aggregation/', filename)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    'Results/Sequence_Aggregation/'.rmtree(file_path)
            except Exception as e:
                print(f"Failed to delete {file_path}. Reason: {e}")






# Use a video to display how the movement appears in the original view.
def processInOriginalView(robot_original, target_original, barrier_original, path, min_xlim,max_xlim, min_ylim, max_ylim, fig, flag_save):

    fig.clf()
    plt.xlim(min_xlim, max_xlim)
    plt.ylim(min_ylim, max_ylim)
    plt.ion()

    robot = [0, 0]
    robotPath = [[robot[0], robot[1]]]

    robotPath_L = []

    sequenceIndex = 0



    for i in range(len(target_original)):
        test = plt.Circle((target_original[i].x + 0.5, target_original[i].y + 0.5), 0.5, facecolor='red', edgecolor='red', alpha=1)
        fig.gca().add_patch(test)

    for i in range(len(barrier_original)):
        test = plt.Circle((barrier_original[i].x + 0.5, barrier_original[i].y + 0.5), 0.5, facecolor='black', edgecolor='black', alpha=1)
        fig.gca().add_patch(test)

    for i in range(len(robot_original)):
        test = plt.Circle((robot_original[i].x + 0.5, robot_original[i].y + 0.5), 0.5, facecolor='green', edgecolor='green', alpha=1)
        fig.gca().add_patch(test)

        robotPath_L.append([[robot_original[i].x, robot_original[i].y]])

    if flag_save:
        filename = 'Results/Sequence_Original/' + str(sequenceIndex) + '.png'
        plt.savefig(filename)

    robot_stop = []

    ta_exclude = set()
    ba_exclude = set()

    for p in range(len(path)):

        plt.pause(0.2)

        fig.clf()

        plt.xlim(min_xlim, max_xlim)
        plt.ylim(min_ylim, max_ylim)

        robot[0] = robot[0] + path[p][0]
        robot[1] = robot[1] + path[p][1]

        robotPath.append([robot[0], robot[1]])

        for i in range(len(robot_original)):

            if i not in robot_stop:
                tempx = robotPath_L[i][-1][0] + path[p][0]
                tempy = robotPath_L[i][-1][1] + path[p][1]

                robotPath_L[i].append([tempx, tempy])

        for i in range(len(robot_original)):
            for obj in target_original:
                if obj.x == robotPath_L[i][-1][0] and obj.y == robotPath_L[i][-1][1]:
                    ta_exclude.add(obj)

                    robot_stop.append(i)

            curTa_l = [obj for obj in target_original if obj not in ta_exclude]

        for i in range(len(curTa_l)):
            test = plt.Circle((curTa_l[i].x + 0.5, curTa_l[i].y + 0.5), 0.5, facecolor='red', edgecolor='red', alpha=1)
            fig.gca().add_patch(test)

        for i in range(len(barrier_original)):
            test = plt.Circle((barrier_original[i].x + 0.5, barrier_original[i].y + 0.5), 0.5, facecolor='black', edgecolor='black', alpha=1)
            fig.gca().add_patch(test)

        for i in range(len(robotPath_L)):
            for j in range(len((robotPath_L[i]))):
                if j != len((robotPath_L[i])) - 1:
                    test = plt.Circle((robotPath_L[i][j][0] + 0.5, robotPath_L[i][j][1] + 0.5), 0.5, facecolor='green',
                                      edgecolor='green', alpha=0.2)
                    fig.gca().add_patch(test)
                else:
                    test = plt.Circle((robotPath_L[i][j][0] + 0.5, robotPath_L[i][j][1] + 0.5), 0.5, facecolor='green',
                                      edgecolor='green', alpha=1)
                    fig.gca().add_patch(test)

        if flag_save:
            sequenceIndex = sequenceIndex + 1
            filename = 'Results/Sequence_Original/' + str(sequenceIndex) + '.png'
            plt.savefig(filename)

    plt.pause(0.2)


    if flag_save:
        os.system(f'{ffmpegPath} -framerate 10 -i Results/Sequence_Original/%d.png Results/original.mp4')

        for filename in os.listdir('Results/Sequence_Original/'):
            file_path = os.path.join('Results/Sequence_Original/', filename)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    'Results/Sequence_Original/'.rmtree(file_path)
            except Exception as e:
                print(f"Failed to delete {file_path}. Reason: {e}")

