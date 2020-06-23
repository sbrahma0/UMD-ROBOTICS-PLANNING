# Make sure to have the server side running in V-REP: 
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

import math
import matplotlib.pyplot as plt
from operator import itemgetter



def obstacleCheck(x_co, y_co):
    offset = (0.354 / 2) + 0.5
    check = 0
    # Boundary Conditions
    if (x_co < -5.55 + offset) or (x_co > 5.55 - offset) or (y_co < -5.05 + offset) or (y_co > 5.05 - offset):
        check = 1

    # Circular Objects
    elif ((x_co + 1.65) ** 2 + (y_co + 4.6) ** 2) < (0.405 + offset) ** 2:
        check = 1

    elif ((x_co + 1.17) ** 2 + (y_co + 2.31) ** 2) < (0.405 + offset) ** 2:
        check = 1

    elif ((x_co + 1.17) ** 2 + (y_co - 2.31) ** 2) < (0.405 + offset) ** 2:
        check = 1

    elif ((x_co + 1.65) ** 2 + (y_co - 4.6) ** 2) < (0.405 + offset) ** 2:
        check = 1

    # Circular Table
    elif (((x_co + 4.05) ** 2 + (y_co - 3.25) ** 2) < (0.7995 + offset) ** 2) or (
            ((x_co + 2.46) ** 2 + (y_co - 3.25) ** 2) < (0.7995 + offset) ** 2):
        check = 1

    elif (-4.05 - offset < x_co) and (x_co < -2.45 + offset) and (2.45 - offset < y_co) and (y_co < 4.05 + offset):
        check = 1

    # Long Rectangle touching x-axis (Bottom right)
    elif (1.3 - offset < x_co) and (x_co < 5.55 + offset) and (-5.05 - offset < y_co) and (y_co < -4.7 + offset):  # 1
        check = 1

    elif (-0.81 - offset < x_co) and (x_co < 1.93 + offset) and (-4.7 - offset < y_co) and (y_co < -3.18 + offset):  # 2
        check = 1

    elif (2.24 - offset < x_co) and (x_co < 3.41 + offset) and (-4.7 - offset < y_co) and (y_co < -4.12 + offset):  # 3
        check = 1

    elif (3.72 - offset < x_co) and (x_co < 5.55 + offset) and (-4.7 - offset < y_co) and (y_co < -3.94 + offset):  # 4
        check = 1

    # Other Rectangles
    elif (-1.17 - offset < x_co) and (x_co < -0.26 + offset) and (-1.9 - offset < y_co) and (
            y_co < -0.08 + offset):  # A
        check = 1

    elif (-0.26 - offset < x_co) and (x_co < 1.57 + offset) and (-2.4 - offset < y_co) and (y_co < -1.64 + offset):  # B
        check = 1

    elif (2.29 - offset < x_co) and (x_co < 3.8 + offset) and (-2.38 - offset < y_co) and (y_co < -1.21 + offset):  # C
        check = 1

    elif (4.97 - offset < x_co) and (x_co < 5.55 + offset) and (-3.27 - offset < y_co) and (y_co < -2.1 + offset):  # D
        check = 1

    elif (4.64 - offset < x_co) and (x_co < 5.55 + offset) and (-1.42 - offset < y_co) and (y_co < -0.57 + offset):  # E
        check = 1

    elif (4.97 - offset < x_co) and (x_co < 5.55 + offset) and (-0.57 - offset < y_co) and (y_co < 0.6 + offset):  # F
        check = 1

    elif (1.89 - offset < x_co) and (x_co < 5.55 + offset) and (1.16 - offset < y_co) and (y_co < 1.92 + offset):  # G
        check = 1

    elif (2.77 - offset < x_co) and (x_co < 3.63 + offset) and (3.22 - offset < y_co) and (y_co < 5.05 + offset):  # H
        check = 1

    elif (4.28 - offset < x_co) and (x_co < 4.71 + offset) and (4.14 - offset < y_co) and (y_co < 5.05 + offset):  # I
        check = 1

    return check


def checkPoint(node, points):
    check = 0
    for point in points:
        if (((node[0] - point[0]) ** 2 + (node[1] - point[1]) ** 2) - 0.1 ** 2 < 0):
            return True
    return False

def hCost(node, goalNode):
    return (((node[0] - goalNode[0]) ** 2 + (node[1] - goalNode[1]) ** 2) ** (1 / 2))


def cost(node, parentNode):
    return (((node[0] - parentNode[0]) ** 2 + (node[1] - parentNode[1]) ** 2) ** (1 / 2))


def action(node, leftwheel, rightwheel, currtheta):
    wl_rad = 0.076 / 2
    dist_bw_wl = 0.23

    temporary_x = node[0]
    temporary_y = node[1]
    temporary_theta = currtheta
    for i in range(1, 101):
        temporary_x = temporary_x + (wl_rad / 2) * (rightwheel + leftwheel) * (math.cos(temporary_theta)) * (2 / 100)
        temporary_y = temporary_y + (wl_rad / 2) * (leftwheel + rightwheel) * (math.sin(temporary_theta)) * (2 / 100)
        temporary_theta = temporary_theta + (wl_rad / dist_bw_wl) * (rightwheel - leftwheel) * (2 / 100)
        if obstacleCheck(round(temporary_x, 1), round(temporary_y, 1)) == True:
            # print('asdnjask bjasbjsabd')
            return math.floor(temporary_x*100)/100, math.floor(temporary_y*100)/100, temporary_theta

    # print(round(temporary_x, 2), round(temporary_y, 2), temporary_theta)
    return math.floor(temporary_x*100)/100, math.floor(temporary_y*100)/100, temporary_theta


def updateCost(child_node, goal_node, node, theta, action, all_nodes, cost_dict, p_id, action_dict, theta_dict):
    heucost = hCost(node, goal_node)  # Calculate Heuristic cost between the current point and goal point
    cost_to_come = cost(node, child_node)
    if (child_node[0], child_node[1]) == (node[0], node[1]):

        return all_nodes, cost_dict, p_id, action_dict, theta_dict
    # print(child_node, node)
    if (node[0], node[1]) not in list(
            cost_dict.keys()):  # If the child node is not in the dictionary i.e. does not have any cost assigned to it
        # print(node+[cost_dict[(child_node[0], child_node[1], lasttheta)] + cost_to_come + heucost] + [theta])
        all_nodes.append(
            node + [cost_dict[(child_node[0], child_node[1])] + cost_to_come + heucost])  # Then add the child node to the all_nodes list
        cost_dict[(node[0], node[1])] = cost_dict[(
        child_node[0], child_node[1])] + cost_to_come  # Then add child node in the cost_dict with cost = current code + Cost of the step
        p_id[(node[0], node[1])] = (
        child_node[0], child_node[1])  # Then add child node in the p_id with its corresponding parent
        action_dict[(node[0], node[1])] = action
        theta_dict[(node[0], node[1])] = theta

    # If the child node is already in the dictionary i.e. have a cost already assigned to it
    elif cost_dict[(node[0], node[1])] + heucost > cost_dict[(
    child_node[0], child_node[1])] + cost_to_come + heucost:  # If the previous cost is greater than the current cost for the child node
        # print('yeesssssssssssssssssss')
        # print('Update Node',node, cost_dict[(child_node[0], child_node[1], child_node[2])] + cost_to_come + heucost)
        # print('Lastcost', cost_dict[(node[0], node[1],  node[2])] + heucost)
        all_nodes.remove(node + [
            cost_dict[(node[0], node[1])] + heucost])  # Remove the previous data of child node from the all_nodes
        all_nodes.append(
            node + [cost_dict[(child_node[0], child_node[1])] + cost_to_come + heucost])  # Add new data of the child node to the all_nodes
        cost_dict[(node[0], node[1])] = cost_dict[(
        child_node[0], child_node[1])] + cost_to_come  # Update new cost of the child node in the cost_dict dictionary
        p_id[(node[0], node[1])] = (
        child_node[0], child_node[1])  # Update new parent of the child node in the p_id dictionary
        action_dict[(node[0], node[1])] = action
        theta_dict[(node[0], node[1])] = theta

    # print('total', cost_dict[(child_node[0], child_node[1], child_node[2])] + cost_to_come + heucost)
    all_nodes = sorted(all_nodes, key=itemgetter(2))  # Sorting the all_nodes list upon cost
    # print(all_nodes)
    return all_nodes, cost_dict, p_id, action_dict, theta_dict  # Returning the updated all_nodes, cost_dict and p_id

no = 0

def astarTurtle(start_point, goal_point, initial_theta):
    currentnode = [start_point[0], start_point[1]]  # Variable for storing the current node
    all_nodes = []  # Variable for storing all the unexplored nodes
    cost_dict = {(start_point[0], start_point[1]): 0}  # Variable for storing cost of each node
    closed_nodes = []  # Variable for storing all the explored nodes
    p_id = {}  # Varoable for storing the parent node for each node
    action_dict = {}
    theta_dict = {(start_point[0], start_point[1]): initial_theta}
    n = 0
    # print((currentnode[0] - goal_point[0])**2 + (currentnode[1] - goal_point[1])**2 - 0.01**2 > 0)
    while ((currentnode[0] - goal_point[0]) ** 2 + (currentnode[1] - goal_point[
        1]) ** 2 - 0.4 ** 2 > 0):  # Looping while the current node is not equal to the goal node

        # Calculating the child node in right direction, cost for moving right is 1
        c1_x, c1_y, c1_theta = action(currentnode, 5, 0, theta_dict[(currentnode[0], currentnode[1])])
        if obstacleCheck(c1_x, c1_y) == False and ([c1_x, c1_y] not in closed_nodes) and (
                checkPoint([c1_x, c1_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = updateCost(currentnode, goal_point,
                                                                                [c1_x, c1_y], c1_theta, [5, 0],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        # Calculating the child node in right direction, cost for moving right is 1
        c2_x, c2_y, c2_theta = action(currentnode, 0, 5, theta_dict[(currentnode[0], currentnode[1])])
        if obstacleCheck(c2_x, c2_y) == False and ([c2_x, c2_y] not in closed_nodes) and (
                checkPoint([c2_x, c2_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = updateCost(currentnode, goal_point,
                                                                                [c2_x, c2_y], c2_theta, [0, 5],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        # Calculating the child node in right direction, cost for moving right is 1
        c3_x, c3_y, c3_thta = action(currentnode, 5, 5, theta_dict[(currentnode[0], currentnode[1])])
        if obstacleCheck(c3_x, c3_y) == False and ([c3_x, c3_y] not in closed_nodes) and (
                checkPoint([c3_x, c3_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = updateCost(currentnode, goal_point,
                                                                                [c3_x, c3_y], c3_thta, [5, 5],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        # Calculating the child node in right direction, cost for moving right is 1
        c4_x, c4_y, c4_theta = action(currentnode, 10, 0, theta_dict[(currentnode[0], currentnode[1])])
        if obstacleCheck(c4_x, c4_y) == False and ([c4_x, c4_y] not in closed_nodes) and (
                checkPoint([c4_x, c4_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = updateCost(currentnode, goal_point,
                                                                                [c4_x, c4_y], c4_theta, [10, 0],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        # Calculating the child node in right direction, cost for moving right is 1
        c5_x, c5_y, c5_theta = action(currentnode, 0, 10, theta_dict[(currentnode[0], currentnode[1])])
        if obstacleCheck(c5_x, c5_y) == False and ([c5_x, c5_y] not in closed_nodes) and (
                checkPoint([c5_x, c5_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = updateCost(currentnode, goal_point,
                                                                                [c5_x, c5_y], c5_theta, [0, 10],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        # Calculating the child node in right direction, cost for moving right is 1
        c6_x, c6_y, c6_theta = action(currentnode, 10, 10, theta_dict[(currentnode[0], currentnode[1])])
        if obstacleCheck(c6_x, c6_y) == False and ([c6_x, c6_y] not in closed_nodes) and (
                checkPoint([c6_x, c6_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = updateCost(currentnode, goal_point,
                                                                                [c6_x, c6_y], c6_theta, [10, 10],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        # Calculating the child node in right direction, cost for moving right is 1
        c7_x, c7_y, c7_theta = action(currentnode, 5, 10, theta_dict[(currentnode[0], currentnode[1])])
        if obstacleCheck(c7_x, c7_y) == False and ([c7_x, c7_y] not in closed_nodes) and (
                checkPoint([c7_x, c7_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = updateCost(currentnode, goal_point,
                                                                                [c7_x, c7_y], c7_theta, [5, 10],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        # Calculating the child node in right direction, cost for moving right is 1
        c8_x, c8_y, c8_theta = action(currentnode, 10, 5, theta_dict[(currentnode[0], currentnode[1])])
        if obstacleCheck(c8_x, c8_y) == False and ([c8_x, c8_y] not in closed_nodes) and (
                checkPoint([c8_x, c8_y], closed_nodes) == False):
            all_nodes, cost_dict, p_id, action_dict, theta_dict = updateCost(currentnode, goal_point,
                                                                                [c8_x, c8_y], c8_theta, [10, 5],
                                                                                all_nodes, cost_dict, p_id,
                                                                                action_dict, theta_dict)

        closed_nodes.append(currentnode)  # Adding current node as explored

        if n % 1000 == 0:
            print(n)
            for i in closed_nodes:
                plt.scatter(i[0], i[1], color='b')
            #plt.show()

        # break
        n = n + 1
        # print(all_nodes)
        # print(cost_dict)
        if all_nodes != []:  # If all_nodes not empty
            nextnode = all_nodes.pop(0)  # Take out the next lowest cost node
            no = 0
        else:  # Else if all_nodes is empty i.e. no unexplored node is remaining or cannot go forward
            print('***Sorry no path found***')
            no = 1
            break
            #return 0


        # print(cost_dict)
        # print('Nextnode', nextnode)
        currentnode = [nextnode[0], nextnode[1]]  # Make nextnode our current node for the next iteration
        # print('current_node', currentnode, cost_dict[(currentnode[0], currentnode[1], currentnode[2])])
        # print('Parent', p_id[(currentnode[0], currentnode[1], currentnode[2])])
    #print('fff-',len(closed_nodes))

    path = [(currentnode[0], currentnode[1])]  # Variable for storing the optimal path from the A* Algorithm

    while (path[-1] != (start_point[0], start_point[1])):  # while the parent node not the start node
        path.append(p_id[path[-1]])  # Add next optimal node to the path
        plt.scatter(path[-1][0], path[-1][1], color='r')


    path.reverse()
    path_action = []
    for i in range(1, len(path)):
        path_action.append(action_dict[(path[i][0], path[i][1])])

    return path, path_action, no


strt_x = float(input('Enter Initial x-Coordinates :'))
strt_y = float(input('Enter Initial y-Coordinates :'))
initial_theta_deg = float(input("Enter the initial orientation in degrees- "))
g_x = float(input('Enter Goal x-Coordinates :'))
g_y = float(input('Enter Goal y-Coordinates :'))

initial_theta = (initial_theta_deg * math.pi)/180

if obstacleCheck(strt_x, strt_y) == True:
    print('Initial point is within the obstacle space')
elif obstacleCheck(g_x, g_y) == True:
    print('Goal point is within the obstacle space')
else:
    path, path_action, no = astarTurtle([strt_x, strt_y], [g_x, g_y],initial_theta)
    if no != 1:
        choice = input('Enter y/n to generate the nodes and path plot- ')
        if choice == 'y':
            plt.show()
        print('path- ',path)
        print('action sequence- ',path_action)
        #print('path length-',len(path))
        #print('path action length-', len(path_action))
        print('total number os action steps-',len(path_action))


try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    #res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
    #if res==vrep.simx_return_ok:
    #    print ('Number of objects in the scene: ',len(objs))
    #else:
    #    print ('Remote API function call returned with error code: ',res)
    time = 0
#retrieve motor  handles
    errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_blocking)
    errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_blocking)
    r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_streaming)
    path_speeds = path_action
    for k in path_speeds:
        time = 0
        err_code1 = 1
        err_code2 = 2
        #print(type(k[0]))
        while(err_code1 != 0 and err_code2 != 0):
            err_code1 = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, k[0], vrep.simx_opmode_streaming)
            print(err_code1)

            err_code2 = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, k[1], vrep.simx_opmode_streaming)
            print(err_code2)

        r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

        while(time<=2):

            r, signalValue2 = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

            time = signalValue2 - signalValue

    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')

