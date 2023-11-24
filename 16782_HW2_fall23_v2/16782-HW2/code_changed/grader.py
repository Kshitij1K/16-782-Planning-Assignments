import numpy as np
import pdb
import argparse
import subprocess # For executing c++ executable
import pandas as pd
from timeit import default_timer as timer

plannerList = ["RRT", "RRTCONNECT", "RRTSTAR", "PRM"]

###############################################################
################### Util Functions Below ######################

def convertPIs(aString):
    """ Input: A comma seperated string like "pi/2,pi/4,pi/2,pi/4,pi/2,"
            or 1.2,4,5.3 etc
    Output: string replacing the pis 1.57079,...,...
    """
    if aString[-1] == ",":  # Remove training comma if there is one
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592") # Replace pi with 3.14... if needed
    vecOfStrings = aString.split(",")
    ans = []
    for anExpression in vecOfStrings:
        ans.append(str(eval(anExpression))) # Evaluate expressions if needed
    return ans

###############################################################
################### Main Functions Below ######################


def graderMain(executablePath, gradingCSV):
    # 0.79,2.36,3.2,4.0,5.7,
    # 1.8,2.0,2.4,1.8,2.0,
    # 1.3,1.57,1.3,1.8,0.4,
    # 1.7,1.8,0.79,0.0,5.8,
    # 1.4,0.8,0.0,5.2,4.0,3.0,


    points_str = ["0.8,2.4,3.2,4.0,5.7",
                  "1.8,2.0,2.4,1.8,2.0",
                  "1.3,1.6,1.3,1.8,0.4",
                  "1.7,1.8,0.8,0.0,5.8",
                  "1.4,0.8,0.0,5.0,3.5"]
    
    # problems = [["./map1.txt", "1.570796,0.785398,1.570796,0.785398,1.570796",
    #                             "0.392699,2.356194,3.141592,2.8274328,4.712388"],
    #         ["./map2.txt", "0.392699,2.356194,3.141592",
    #                             "1.570796,0.785398,1.570796"]]

    # generate all permutations of 2 points from points_str
    points_permutations = []
    for i in range(len(points_str)):
        for j in range(i+1, len(points_str)):
            points_permutations.append([points_str[i], points_str[j]])

    problems = [["./map2.txt", points_permutations[0][0], points_permutations[0][1]],
                ["./map2.txt", points_permutations[1][0], points_permutations[1][1]],
                ["./map2.txt", points_permutations[2][0], points_permutations[2][1]],
                ["./map2.txt", points_permutations[3][0], points_permutations[3][1]],
                ["./map2.txt", points_permutations[4][0], points_permutations[4][1]],
                ["./map2.txt", points_permutations[5][0], points_permutations[5][1]],
                ["./map2.txt", points_permutations[6][0], points_permutations[6][1]],
                ["./map2.txt", points_permutations[7][0], points_permutations[7][1]],
                ["./map2.txt", points_permutations[8][0], points_permutations[8][1]],
                ["./map2.txt", points_permutations[9][0], points_permutations[9][1]],
                ["./map2.txt", points_permutations[0][1], points_permutations[0][0]],
                ["./map2.txt", points_permutations[1][1], points_permutations[1][0]],
                ["./map2.txt", points_permutations[2][1], points_permutations[2][0]],
                ["./map2.txt", points_permutations[3][1], points_permutations[3][0]],
                ["./map2.txt", points_permutations[4][1], points_permutations[4][0]],
                ["./map2.txt", points_permutations[5][1], points_permutations[5][0]],
                ["./map2.txt", points_permutations[6][1], points_permutations[6][0]],
                ["./map2.txt", points_permutations[7][1], points_permutations[7][0]],
                ["./map2.txt", points_permutations[8][1], points_permutations[8][0]],
                ["./map2.txt", points_permutations[9][1], points_permutations[9][0]],
                ]


    scores = []
    for aPlanner in [0,1,2,3]:
        print("\nTESTING " + plannerList[aPlanner] + "\n")
        for i, data in enumerate(problems):
            inputMap, startPos, goalPos = [*data]
            numDOFs = len(startPos.split(","))
            outputSolutionFile = "grader_out/tmp.txt"
            commandPlan = "{} {} {} {} {} {} {}".format(
                executablePath,
                inputMap, numDOFs, startPos, goalPos,
                aPlanner, outputSolutionFile)
            print("EXECUTING: " + str(commandPlan))
            commandVerify = "./verifier.out {} {} {} {} {}".format(
                inputMap, numDOFs, startPos, goalPos,
                outputSolutionFile)
            print("EXECUTING: " + str(commandVerify))
            try:
                start = timer()
                subprocess.run(commandPlan.split(" "), check=True) # True if want to see failure errors
                timespent = timer() - start
                returncode = subprocess.run(commandVerify.split(" "), check=False).returncode
                if returncode != 0:
                    print("Returned an invalid solution")
                
                ### Calculate the cost from their solution
                with open(outputSolutionFile) as f:
                    line = f.readline().rstrip()  # filepath of the map
                    solution = []
                    for line in f:
                        solution.append(line.split(",")[:-1]) # :-1 to drop trailing comma
                    solution = np.asarray(solution).astype(float)
                    numSteps = solution.shape[0]

                    ## Cost is sum of all joint angle movements
                    difsPos = np.abs(solution[1:,]-solution[:-1,])
                    cost = np.minimum(difsPos, np.abs(2*np.pi - difsPos)).sum()

                    success = returncode == 0
                    scores.append([aPlanner, inputMap, i, numSteps, cost, timespent, success])
            
                ### Visualize their results
                commandViz = "python visualizer.py grader_out/tmp.txt --gifFilepath=grader_out/grader_{}{}.gif".format(plannerList[aPlanner], i)
                commandViz += " --incPrev=1"
                # subprocess.run(commandViz.split(" "), check=True) # True if want to see failure errors
            except Exception as exc:
                print("Failed: {} !!".format(exc))
                scores.append([aPlanner, inputMap, i, -1, -1, timespent, False])

    ### Save all the scores into a csv to compute overall grades
    df = pd.DataFrame(scores, columns=["planner", "mapName", "problemIndex", "numSteps", "cost", "timespent", "success"])
    df.to_csv(gradingCSV, index=False)
            

if __name__ == "__main__":
    graderMain("./planner.out", "grader_out/grader_results.csv")