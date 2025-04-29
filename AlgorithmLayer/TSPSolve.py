# Use the ant colony algorithm to solve the travelling salesman problem.

import sys
sys.path.append('../BaseLayer/')
import BaseUnits

import GridMap4BFS
import numpy as np
import copy

from itertools import accumulate
import random

import UpdateD
from multiprocessing import Pool

from functools import partial

sys.path.append('../PresentationLayer/')




def TSPSolve(D, targetList_aggregation, barrierList_aggregation,num_initRobot, num_initTarget):


    m = 160         #Number of ants
    Alpha = 0.8    #  importance of pheromones
    Beta = 3        # importance of the heuristic factor
    Rho = 0.1       # Pheromone evaporation coefficient
    NC_max = 100    # Maximum number of iterations
    Q = 100         # Pheromones increase the intensity coefficient

    # Variable initialisation
    n = D.shape[0]
    allElements = list(range(n))
    # Eta = 1 / D                                             # Heuristic, set to the reciprocal of the distance
    Tau = np.ones((n, n))                                   # Pheromone matrix
    NC = 1
    Path = np.zeros((m,n),dtype=int)
    Tabu = []                                               # Taboo table, which stores the paths that have been travelled

    R_best = np.zeros((NC_max,n))                           # The best path for all generations.
    L_best = np.ones((NC_max,1)) * 1 / np.finfo(float).eps  # The length of the optimal path for each generation.
    L_ave = np.zeros((NC_max,1))                            # The average length of paths of each generation.



    while NC <= NC_max:

        Path[:,[0]] = 0
        Path[:,[1]] = 1

        L = np.zeros((m,1))

        flag_failedAnt = []

        pool = Pool(processes = 80)


        for i in range(m):





            partial_callback = partial(callback_antTravel, L = L, flag_failedAnt = flag_failedAnt, Path = Path, i = i)

            pool.apply_async(func=antTravel,
                             args=(D, targetList_aggregation, Path, barrierList_aggregation,  Tau, NC, i, Alpha, Beta, num_initRobot, num_initTarget,),
                             callback=partial_callback, error_callback=print)

        pool.close()
        pool.join()


        if len(flag_failedAnt) == m:
            print("\033[1;31mNone of the ants found a feasible path, optimization failed!\033[0m")
            return R_best, L_best, L_ave


        if NC >= 2:

            Path[0] = R_best[NC - 1 - 1]
            L[0] = L_best[NC - 1 - 1]

            if 0 in flag_failedAnt:
                flag_failedAnt = [x for x in flag_failedAnt if x != 0]


        length_best  = 1 / np.finfo(float).eps
        path_best_index = -1
        length_total = 0
        counter_succeed = 0
        
        for p in range(L.shape[0]):
          if p not in flag_failedAnt:
            if L[p,0] < length_best:
              length_best = L[p,0]
              path_best_index = p
            
            length_total = length_total + L[p,0]
            counter_succeed = counter_succeed + 1
            

            
        L_best[NC - 1] = length_best
        R_best[NC - 1] = Path[path_best_index]
        L_ave[NC - 1] = length_total / counter_succeed
        
        print('Iteraion ' + str(NC) + " : the length of best path:" + str(L_best[NC - 1]))
        






        NC = NC + 1

        Delta_Tau = np.zeros((n,n))
        for i in range(m):
            if i in flag_failedAnt:
                continue
            else:
                R = Path[i].copy()
                R = R[R != 0]
                for j in range(len(R) - 1):
                    Delta_Tau[R[j], R[j+1]] = Delta_Tau[R[j], R[j+1]] + Q/L[i]

        Tau = (1 - Rho) * Tau + Delta_Tau

        Path = np.zeros((m,n),dtype=int)



    return R_best,L_best,L_ave



def antTravel(D, targetList_aggregation, Path, barrierList_aggregation, Tau, NC, i, Alpha, Beta, num_initRobot, num_initTarget):

    n = D.shape[0]
    allElements = list(range(n))

    Tabu = []
    Tabu.append(0)
    Tabu.append(1)

    D_update = copy.deepcopy(D)
    Path_update = copy.deepcopy(Path)

    curL = []
    for j in range(2, n):
        J = [x for x in allElements if x not in Tabu]
        P = []
        P_index = []
        to_visit = 0


        target_overlap = -1

        if j > 2:
            for k in range(len(J)):
                if targetList_aggregation[J[k] - 2].x == targetList_aggregation[Tabu[-1] - 2].x and \
                        targetList_aggregation[J[k] - 2].y == targetList_aggregation[Tabu[-1] - 2].y:
                    target_overlap = J[k]
                    break


        if target_overlap != -1:
            to_visit = target_overlap

        else:


            path_update = Path_update[i, :].tolist()

            UpdateD.updateD(Tabu, path_update, D_update, targetList_aggregation,
                            barrierList_aggregation)


            Eta = 1 / D_update


            for k in range(len(J)):
                if Eta[Tabu[-1], J[k]] > pow(10, -12):
                    P.append(pow((Tau[Tabu[-1], J[k]]), Alpha) * pow(Eta[Tabu[-1], J[k]], Beta))
                    P_index.append(k)


            if len(P) == 0:
                break

            P = list(accumulate(np.divide(P, sum(P))))


            r = random.random()
            for k in range(len(P)):
                if r < P[k]:
                    to_visit = J[P_index[k]]
                    break

        Path_update[i, j] = to_visit

        curL.append(D_update[Tabu[-1], to_visit])


        for k in range(len(targetList_aggregation)):
            if ((targetList_aggregation[k].parent_robot == targetList_aggregation[to_visit - 2].parent_robot or
                 targetList_aggregation[k].parent_target == targetList_aggregation[
                     to_visit - 2].parent_target) and k != (to_visit - 2)):
                Tabu.append(k + 2)
        Tabu.append(to_visit)


    R = Path_update[i].copy()
    R = R[R != 0]
    flag_failedAnt = False
    if len(R) < (min(num_initRobot, num_initTarget) + 1):

        flag_failedAnt = True

    length = 0
    for k in range(len(R) - 1):

        length = length + D_update[R[k], R[k + 1]]

    return flag_failedAnt, Path_update[i], length




def callback_antTravel(result, L, flag_failedAnt, Path, i):
    flag, path, length = result
    L[i] = length
    if flag:
        flag_failedAnt.append(i)
    Path[i] = path
