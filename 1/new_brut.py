##################################################################################
#######################        SAYAN BRAHMA         ##############################
#######################      PLANNING PROJECT 1     ##############################
##################################################################################

import copy # to copy multidimensional lists and array without passing the refference
import numpy as np
import math

def check_zero(node): # function to get the zero position
    flag = True
    i = 0  # signifies rows
    while i < 3:
        j = 0  # signifies column
        while j < 3:
            # print(node[i][j])
            if node[i][j] == 0:
                flag = False
                break
            j += 1

        if not flag:
            break
        i += 1

    return i,j

def ml(node): # action function to move left
    new_node = copy.deepcopy(node)
    x, y = check_zero(new_node)
    if y > 0:
        new_node[x][y] = copy.deepcopy(new_node[x][y-1])
        new_node[x][y-1] = 0

    return new_node

def mr(node): # action function to move right
    new_node = copy.deepcopy(node)
    x, y = check_zero(new_node)
    if y < 2:
        new_node[x][y] = copy.deepcopy(new_node[x][y+1])
        new_node[x][y+1] = 0
    #print('r', new_node)
    return new_node

def mu(node): # action function to move up
    new_node = copy.deepcopy(node)
    x, y = check_zero(new_node)
    if x > 0:
        new_node[x][y] = copy.deepcopy(new_node[x-1][y])
        new_node[x-1][y] = 0
    #print('u', new_node)
    return new_node

def md(node): # action function to move down
    new_node = copy.deepcopy(node)
    x, y = check_zero(new_node)
    if x < 2:
        new_node[x][y] = copy.deepcopy(new_node[x+1][y])
        new_node[x+1][y] = 0
    #print('d', new_node)
    return new_node

def list_array(lis, fil_name):
    for z1 in range(len(lis)):
        new = [lis[z1][0][0], lis[z1][1][0], lis[z1][2][0], lis[z1][0][1], lis[z1][1][1], lis[z1][2][1], lis[z1][0][2],
               lis[z1][1][2], lis[z1][2][2]]
        fil_name.write(str(new)+'\n')

def bfs(node):  # main function
    nodes = [] # node is just for calling the initial node for the first time
    nodes.append(node)  # stores all the possible nodes
    nodef = nodes.pop(0) # stores the node value based on LIFO
    visitednodes = [] # stores all the visited nodes i.e. the checked nodes
    #print(node)# stores all the parent and child nodes that has been checked with the goal node
    pt = 1 # parent id
    ch = 1 # children id
    dict = {1:0} # for storing the parent and children id
    dist_store = {}
    flag1 =0
    while (nodef != goal_node):
        visitednodes.append(nodef) # stores all the parent and child nodes which are being checked with
        #print('nodesf',nodef)
        #print('nodes',nodes)
        #print('visit',visitednodes)

        if not (ml(copy.deepcopy(nodef)) in nodes) and not (ml(copy.deepcopy(nodef)) in visitednodes):
            nodes.append(ml(copy.deepcopy(nodef))) # stores all the possible nodes from the input node in the loop
            ch = ch+1 # for signifying the possible childs from a node
            dict[ch] = pt
            dist_store[ch] = ml(copy.deepcopy(nodef))

            #print('l')

        if not (mr(copy.deepcopy(nodef)) in nodes) and not (mr(copy.deepcopy(nodef)) in visitednodes):
            nodes.append(mr(copy.deepcopy(nodef)))
            ch = ch + 1
            dict[ch] = pt
            dist_store[ch] = mr(copy.deepcopy(nodef))

            #print('r')

        if not (mu(copy.deepcopy(nodef)) in nodes) and not (mu(copy.deepcopy(nodef)) in visitednodes):
            nodes.append(mu(copy.deepcopy(nodef)))
            ch = ch + 1
            dict[ch] = pt
            dist_store[ch] = mu(copy.deepcopy(nodef))

            #print('u')

        if not (md(copy.deepcopy(nodef)) in nodes) and not (md(copy.deepcopy(nodef)) in visitednodes):
            nodes.append(md(copy.deepcopy(nodef)))
            ch = ch + 1
            dict[ch] = pt
            dist_store[ch] = md(copy.deepcopy(nodef))

            #print('d')
        #print('nodes- ',nodes)


        nodef = nodes.pop(0) # stores the last value from the nodes and send it to the loop for comparison
        if nodef == goal_node:
            flag1 = 1

        #print(pt)
        pt = pt + 1



    visitednodes.append(nodef) # stores the visited node i.e., the checked nodes
    #print('dict= ',dict)
    res = [len(visitednodes)-1] # stores the visited nodes's length

    #print('visited node- ',visitednodes) # prints all the visited nodes- to be stored in nodes text file
    #print('dict:- ',dict)
    if flag1 == 0:            # need to placed in right position
        print ("No solution")
        return visitednodes, dict
    #print('res',res)

    while (res[0] != 0): # loop continues till it finds the parent node
        #print(res[0])
        res = [dict[res[0]]] + res # stores all the successive parent and child nodes

    #print ('node numbers resulting to goal node- ',res)
    #for value in res: # for extracting the specified parent and child nodes from the visitednode list
    #   print(visitednodes[value])


    return nodes, visitednodes, dict, res, pt, dist_store

snode = input('Enter an input configuration- ').split(',')
l1 = []
for i in snode:
    l1.append(int(i))
init_node = [[l1[0], l1[1], l1[2]], [l1[3], l1[4], l1[5]], [l1[6], l1[7], l1[8]]]
init_node2 = [l1[0], l1[1], l1[2], l1[3], l1[4], l1[5], l1[6], l1[7], l1[8]]

goal_node = [[1,2,3],[4,5,6],[7,8,0]] # goal node
inpo = 0
for cp in range(8):  # for checking the inversion to check whether the puzzle is solvable or not
    cp1 = cp+1
    for cp1 in range(cp1,9):
        if (init_node2[cp]>0) and (init_node2[cp1]>0) and (init_node2[cp]>init_node2[cp1]):
            if init_node2[cp1] == 0:
                inpo = inpo-1
            else:
                inpo = inpo+1


if inpo%2 == 0:
    print("solvable")

    if init_node != goal_node:

        u, v, d, r, p, f = bfs(init_node)
        print('All nodes- ',v+u)
        #print((v+u)[0])
        print('Visited nodes- ',v) # visited node till the goal node arrives
        #print(d) # dictionary
        #print(p)

        bt = []
        j = p
        add = []
        while j!= 1:
            bt.insert(0,f[j])
            c = copy.deepcopy(d[j])
            j = copy.deepcopy(c)
            add.append(j)
        bt.insert(0,init_node)
        print('node path- ',bt) # prints the node path
        node_info = []
        all_node_info = []
        l = add.pop(0)

        for z in range(1,len(v)):
            node_info.append([z, d[z], d[z]])
        node_info.append([p, l, l])
        print('visited node info- ',node_info)

        for z1 in range(1, len(d)+1):
            all_node_info.append([z1, d[z1], d[z1]])
        print('All node info- ',all_node_info)
        #print('dict len- ',len(d))
        #print('dict-',d)




        fh1 = open('AllNodes.txt','w')
        fh2 = open('AllNodesInfo.txt','w')
        fh3 = open('NodePath.txt','w')

        nodes_pr = np.array(copy.deepcopy(v+u))
        nodepath_pr = np.array(copy.deepcopy(bt))

        list_array(nodes_pr, fh1)
        list_array(nodepath_pr, fh3)
        for s in range(len(all_node_info)):
            fh2.write(str(all_node_info[s])+'\n')

        fh1.close()
        fh2.close()
        fh3.close()

    elif init_node == goal_node:

        print('Visited nodes- ', init_node)
        print('node path- ', init_node)
        print('node info- ', [1, 0, 0])

        fh1 = open('AllNodes.txt', 'w')
        fh2 = open('AllNodesInfo.txt', 'w')
        fh3 = open('NodePath.txt', 'w')

        fh1.write(str(init_node))
        fh3.write(str(init_node))
        fh2.write('[1, 0, 0]')

        fh1.close()
        fh2.close()
        fh3.close()
else:
    print('Unsolvable')
    fh1 = open('AllNodes.txt', 'w')
    fh2 = open('AllNodesInfo.txt', 'w')
    fh3 = open('NodePath.txt', 'w')
    fh1.close()
    fh2.close()
    fh3.close()
