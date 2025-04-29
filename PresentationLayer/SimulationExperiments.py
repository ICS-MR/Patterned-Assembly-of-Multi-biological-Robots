# Simulation experiments to verify the performance of the algorithm.
import math
import sys
sys.path.append('../BaseLayer/')
import BaseUnits
import GridMap4BFS
import MapInit
sys.path.append('../AlgorithmLayer/')
import FieldConversion
import AdjacencyMatrixCompletion
import TSPSolve


import matplotlib.pyplot as plt
import numpy as np

import pickle

import Visualization

import ExperimentInitialization

import warnings





if __name__ == "__main__":

    warnings.filterwarnings("ignore")

    argument = 'T'    # Matching parameters to determine which experiment to run. can be set as X_1, X_2, X_3. I, T, A, H, HEART, HIT, X_lessrobot, X_morerobot

    OperationalSettings = ['rerunAll', 'noRerun']
    OperationalSetting = OperationalSettings[1]
    

    experimentDicts = vars(ExperimentInitialization)

    if argument in experimentDicts and isinstance(experimentDicts[argument], dict):
        curExperiment = experimentDicts[argument]

    else:
        print("\033[1;31mThe experiment was not found, please check the input parameters!\033[0m")
        sys.exit()


########################################################################################################################

    if OperationalSetting == 'rerunAll':

        ro_l, ta_l, ba_l = MapInit.mapInit(curExperiment['ro'], curExperiment['ta'], curExperiment['ba'])


        ro_a, ta_a, ba_a = FieldConversion.fieldConversion(ro_l, ta_l, ba_l)

        print("The aggregate view is created.")


        
        Visualization.originalViewPlot(ro_l, ta_l, ba_l, -50, 50, -50, 50, False)

        Visualization.aggregationViewPlot(ta_a, ba_a, -50, 50, -50, 50, False)

        D, D_constant = AdjacencyMatrixCompletion.adjacencyMatrixCompletion(ro_a, ta_a, ba_a)

        print("Adjacency matrix initialization is complete.")


        R_best, L_best, L_ave = TSPSolve.TSPSolve(D, ta_a, ba_a, len(ro_l), len(ta_l))

        Visualization.iterativeResultPlot(L_ave, L_best, False)

        path, totalLength = Visualization.detailPathCalc(R_best, ro_a, ta_a, ba_a)
        # print(totalLength)


        data = {
            'ro_l': ro_l,
            'ta_l': ta_l,
            'ba_l': ba_l,
            'ro_a': ro_a,
            'ta_a': ta_a,
            'ba_a': ba_a,
            'D': D,
            'D_constant': D_constant,
            'R_best': R_best,
            'L_best': L_best,
            'L_ave': L_ave,
            'path': path
        }
        filename = 'Results/' + argument + '.pickle'
        with open(filename,'wb') as file:
            pickle.dump(data, file)

        fig = plt.figure(figsize=(6,6))

        Visualization.processInAggregationView(ta_a, ba_a, path, -50, 50, -50, 50, fig, False)
        Visualization.processInOriginalView(ro_l, ta_l, ba_l, path, -50, 50, -50, 50, fig, False)

########################################################################################################################

    if OperationalSetting == 'noRerun':
        filename = 'Results/' + argument + '.pickle'
        with open(filename, 'rb') as file:
            loaded_data = pickle.load(file)

        ro_l = loaded_data['ro_l']
        ta_l = loaded_data['ta_l']
        ba_l = loaded_data['ba_l']
        ro_a = loaded_data['ro_a']
        ta_a = loaded_data['ta_a']
        ba_a = loaded_data['ba_a']
        D = loaded_data['D']
        D_constant = loaded_data['D_constant']
        R_best = loaded_data['R_best']
        L_best = loaded_data['L_best']
        L_ave = loaded_data['L_ave']
        path = loaded_data['path']


        Visualization.originalViewPlot(ro_l, ta_l, ba_l, -50, 50, -50, 50, True)





        Visualization.aggregationViewPlot(ta_a, ba_a, -50, 50, -50, 50, True)

        Visualization.iterativeResultPlot(L_ave, L_best, True)

        fig = plt.figure(figsize=(6, 6))

        Visualization.processInAggregationView(ta_a, ba_a, path, -50, 50, -50, 50, fig, False)
        Visualization.processInOriginalView(ro_l, ta_l, ba_l, path, -50, 50, -50, 50, fig, False)
