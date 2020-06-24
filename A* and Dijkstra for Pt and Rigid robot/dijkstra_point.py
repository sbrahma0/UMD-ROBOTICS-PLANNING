from operator import itemgetter
import pygame as pg

def obstacle_check(x_co, y_co, r):
    flag = 0
    if ((x_co - 190 / r) ** 2 + (y_co - 130 / r) ** 2 - (
            15 / r) ** 2) <= 0:  # Semi-algebraic equation for circle
        flag = 1
    elif (((x_co - 140 / r) / (15 / r)) ** 2 + (
            (y_co - 120 / r) / (6 / r)) ** 2) <= 1:  # Semi-algebraic equation for ellipse
        flag = 1
    elif (y_co - 112.5 / r <= 0) and (y_co - 67.5 / r >= 0) and (x_co - 50 / r >= 0) and (
            x_co - 100 / r <= 0):  # Half plane equations for square
        flag = 1
    # Half plane equations for the polygon
    elif ((y_co - 56 / r) - ((56.0 - 15.0) / (125 - 150)) * (x_co - 125 / r)) >= 0 and (
            (y_co - 52 / r) - ((52.0 - 56.0) / (163 - 125)) * (x_co - 163 / r)) <= 0 and (
            (y_co - 52 / r) - ((52.0 - 15.0) / (163 - 150)) * (
            x_co - 163 / r)) >= 0:  # and (y_co - 15/r) >= 0:
        flag = 1
    elif ((y_co - 90 / r) - ((52.0 - 90.0) / (163 - 170)) * (x_co - 170 / r)) <= 0 and (
            (y_co - 52 / r) - ((52.0 - 90.0) / (193 - 170)) * (x_co - 193 / r)) <= 0 and (y_co - 52 / r) >= 0:
        flag = 1
    elif ((y_co - 15 / r) - ((52.0 - 15.0) / (193 - 173)) * (x_co - 173 / r)) >= 0 and (
            y_co - 15 / r) >= 0 and (y_co - 52 / r) <= 0 and (
            (y_co - 52 / r) - ((52.0 - 15.0) / (163 - 150)) * (x_co - 163 / r)) <= 0:
        flag = 1

    if flag == 1:
        return 1  # Returns 1 if the given coordiante is in obstacle space
    else:
        return 0  # Returns 0 if the given coordiante is not in obstacle space



def creat_map(screen, w, h, r):
    obstacles = []
    m = 1
    for x_co in range(0, round(250 / r)):
        for y_co in range(0, round(150 / r)):

            flag = obstacle_check(x_co + 0.5, y_co + 0.5, r)

            if flag == 1:
                obstacles.append([x_co, y_co])
                color = (0, 0, 0)

            else:
                l1 = [obstacle_check(x_co + 0.005, y_co + 0.005, r),
                      obstacle_check(x_co + 0.995, y_co + 0.995, r)]
                l2 = [obstacle_check(x_co + 1 - 0.005, y_co + 0.005, r),
                      obstacle_check(x_co + 1 - 0.995, y_co + 0.995, r)]

                if 1 in l1:
                    color = (0, 0, 0)
                    obstacles.append([x_co, y_co])
                elif 1 in l2:
                    color = (0, 0, 0)
                    obstacles.append([x_co, y_co])
                else:
                    color = (250, 250, 0)
            pg.draw.rect(screen, color, [(m + w) * (x_co) + m,
                                             (m + h) * (int(150 / r) - y_co) + m, w, h])

    pg.display.flip()
    return obstacles


def update_cost(cnode, node, cost, g_cost, open_list, cost_dict, parentid):
    if (node[0], node[1]) not in list(cost_dict.keys()):
        open_list.append(node + [cost + g_cost])
        cost_dict[(node[0], node[1])] = cost + g_cost
        parentid[(node[0], node[1])] = (cnode[0], cnode[1])

    elif cost_dict[(node[0], node[1])] > cost + g_cost:
        open_list.remove(node + [cost_dict[(node[0], node[1])]])
        open_list.append(node + [cost + g_cost])
        cost_dict[(node[0], node[1])] = cost + g_cost
        parentid[(node[0], node[1])] = (cnode[0], cnode[1])

    open_list = sorted(open_list, key=itemgetter(2))

    return open_list, cost_dict, parentid


def update_anim(screen, node, h, w, r, last, first):
    if node[0] != int(250 / r) and node[1] != int(150 / r):
        m = 1
        pg.draw.rect(screen, (0, 255, 0),
                         [(m + w) * node[0] + m, (m + h) * (int(150 / r) - node[1]) + m,
                          w, h])
        #pg.draw.rect(screen, (255, 0, 0), [(m + w) * last[0] + m, (m + h) * (int(150 / r) - last[1]) + m, w, h])

        pg.draw.rect(screen, (255, 0, 0), [(m + w) * first[0] + m, (m + h) * (int(150 / r) - first[1]) + m, w, h])

        pg.display.flip()


def final_anim(screen, nodes, h, w, r):
    m = 1
    for node in nodes:
        if node[0] == int(250 / r) or node[1] == int(150 / r):
            pg.draw.rect(screen, (255, 0, 0), [(m + w) * (node[0]-1) + m,
                                                   (m + h) * (int(150 / r) - (node[1]+1)) + m, w,
                                                   h])
        else:
            pg.draw.rect(screen, (255, 0, 0), [(m + w) * node[0] + m,
                                               (m + h) * (int(150 / r) - node[1]) + m, w,
                                               h])
    pg.display.flip()


def dijkstra_point(start_point, goal_point, obstacles, w, h, r, screen):
    currentnode = [start_point[0], start_point[1]]
    currentcost = 0
    first = [start_point[0], start_point[1]]
    open_list = []
    cost_dict = {}
    closed_list = []
    parentid = {}

    while (currentnode != [goal_point[0], goal_point[1]]):
        pg.event.pump()
        if (currentnode[0] + 1 <= 250 / r) and ([currentnode[0] + 1, currentnode[1]] not in obstacles) and (
                [currentnode[0] + 1, currentnode[1]] not in closed_list):# Move right action
            open_list, cost_dict, parentid = update_cost(currentnode, [currentnode[0] + 1, currentnode[1]], currentcost,
                                                       1, open_list, cost_dict, parentid)

        if (currentnode[0] - 1 >= 0) and ([currentnode[0] - 1, currentnode[1]] not in obstacles) and (
                [currentnode[0] - 1, currentnode[1]] not in closed_list): # move left action
            open_list, cost_dict, parentid = update_cost(currentnode, [currentnode[0] - 1, currentnode[1]], currentcost,
                                                       1, open_list, cost_dict, parentid)

        if (currentnode[1] + 1 <= 150 / r) and ([currentnode[0], currentnode[1] + 1] not in obstacles) and (
                [currentnode[0], currentnode[1] + 1] not in closed_list): # move up action
            open_list, cost_dict, parentid = update_cost(currentnode, [currentnode[0], currentnode[1] + 1], currentcost,
                                                       1, open_list, cost_dict, parentid)

        if (currentnode[1] - 1 >= 0) and ([currentnode[0], currentnode[1] - 1] not in obstacles) and (
                [currentnode[0], currentnode[1] - 1] not in closed_list): # move down action
            open_list, cost_dict, parentid = update_cost(currentnode, [currentnode[0], currentnode[1] - 1], currentcost,
                                                       1, open_list, cost_dict, parentid)

        if (currentnode[0] + 1 <= 250 / r) and (currentnode[1] + 1 <= 150 / r) and (
                [currentnode[0] + 1, currentnode[1] + 1] not in obstacles) and (
                [currentnode[0] + 1, currentnode[1] + 1] not in closed_list): # move up-right action
            open_list, cost_dict, parentid = update_cost(currentnode, [currentnode[0] + 1, currentnode[1] + 1],
                                                       currentcost, (2) ** (1 / 2), open_list, cost_dict, parentid)

        if (currentnode[0] - 1 >= 0) and (currentnode[1] + 1 <= 150 / r) and (
                [currentnode[0] - 1, currentnode[1] + 1] not in obstacles) and (
                [currentnode[0] - 1, currentnode[1] + 1] not in closed_list): # move down-right action
            open_list, cost_dict, parentid = update_cost(currentnode, [currentnode[0] - 1, currentnode[1] + 1],
                                                       currentcost, (2) ** (1 / 2), open_list, cost_dict, parentid)

        if (currentnode[0] + 1 <= 250 / r) and (currentnode[1] - 1 >= 0) and (
                [currentnode[0] + 1, currentnode[1] - 1] not in obstacles) and (
                [currentnode[0] + 1, currentnode[1] - 1] not in closed_list): # move up-left action
            open_list, cost_dict, parentid = update_cost(currentnode, [currentnode[0] + 1, currentnode[1] - 1],
                                                       currentcost, (2) ** (1 / 2), open_list, cost_dict, parentid)

        if (currentnode[0] - 1 >= 0) and (currentnode[1] - 1 >= 0) and (
                [currentnode[0] - 1, currentnode[1] - 1] not in obstacles) and (
                [currentnode[0] - 1, currentnode[1] - 1] not in closed_list):
            open_list, cost_dict, parentid = update_cost(currentnode, [currentnode[0] - 1, currentnode[1] - 1],
                                                       currentcost, (2) ** (1 / 2), open_list, cost_dict, parentid)

        closed_list.append(currentnode)
        if first[0] == int(250 / r) or first[1] == int(150 / r):
            update_anim(gameDisplay, currentnode, h, w, r, (goal_point[0] - 1, goal_point[1] - 1),
                        (first[0] - 1, first[1] - 1))
        else:
            update_anim(gameDisplay, currentnode, h, w, r, (goal_point[0] - 1, goal_point[1] - 1), (first[0], first[1]))

        if open_list != []:
            nextnode = open_list.pop(0)
        else:
            print('***No Path Found***')
            pg.quit()
            return 0

        currentnode = [nextnode[0], nextnode[1]]
        currentcost = nextnode[2]

        done = False
        for event in pg.event.get():
            if event.type == pg.KEYDOWN:
                if event.key == pg.K_ESCAPE: done = True
            elif event.type == pg.QUIT:
                done = True
        if done:
            print('***Force Closed Processing***')
            pg.quit()
            return 0

    path = [(goal_point[0], goal_point[1])]
    while (path[-1] != (start_point[0], start_point[1])):
        path.append(parentid[path[-1]])

    path.reverse()
    final_anim(gameDisplay, path, h, w, r)
    print(path)
    pg.time.wait(10000)
    pg.quit()




r = float(input('Enter rolution/grid-size- '))
s = input('Enter the x,y of the starting point(separated by comma)- ').split(',')
s1 = []
for d in s:
    s1.append(float(d))

sx = float(s1[0])
sy = float(s1[1])


g = input('Enter the x,y of the goal point(separated by comma)- ').split(',')
g1 = []
for d1 in g:
    g1.append(int(d1))

gx = float(g1[0])
gy = float(g1[1])



if r != 0:
    pg.init()
    gameDisplay = pg.display.set_mode((int(760 + 250 / r), int(460 + 150 / r)))
    gameDisplay.fill((0, 0, 0))
    pg.display.flip()
    obs = creat_map(gameDisplay, 750 / (250 / r), 450 / (150 / r), r)
    if ([int(sx / r), int(sy / r)] in obs):
        pg.quit()
        print('sorry start point is within the obstacle space')
    elif sx < 0 or sx > 250 or sy < 0 or sy > 150:
        pg.quit()
        print('sorry start point is out of bounds')
    elif ([int(gx / r), int(gy / r)] in obs):
        pg.quit()
        print('soory goal point is within the obstacle space')
    elif gx < 0 or gx > 250 or gy < 0 or gy > 150:
        pg.quit()
        print('sorry goal point is out of boundaries')
    else:
        dijkstra_point((int(sx / r), int(sy / r)), (int(gx / r), int(gy / r)), obs,
                       750 / (250 / r), 450 / (150 / r), r, gameDisplay)

else:
    pg.quit()
    print('Sorry this is not a feasible rolution')
