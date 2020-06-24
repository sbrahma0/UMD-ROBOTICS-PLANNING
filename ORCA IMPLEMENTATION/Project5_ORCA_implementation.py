#!/usr/bin/env python
# coding: utf-8

# In[2]:


##########################################################################################
######################## Project-5 ORCA Implementation ###################################
# Group- 9
# NIKHIL MEHRA - 116189941
# SAKET SESHADRI GUDIMETLA - 116332293
# SAYAN BRAHMA - 116309165
##########################################################################################

import itertools
from numpy import dot, clip, array, sqrt, rint, linspace, pi, cos, sin, copysign
from numpy.linalg import det
import pygame
import random

################################################################################
# Class Name - Line
# Arguments for object Initialization - point and direction
# point: any (x,y) point on the line
# direction: (x, y, z) vector 
################################################################################

class Line(object):

    def __init__(self, point, direction):
      
        self.pt = array(point)
        self.dir = normalized(array(direction))

    def __repr__(self):
        return "Line(%s, %s)" % (self.point, self.direction)
    
################################################################################
# Class Name - Bot
# Arguments for object Initialization - pos, vel, rad, v_max, v_pref
# pos - (x,y) position of the velocity in the Cartesian space
# vel - (velx, vely) range of velocity the robot can take
# rad - radius of the bot
# v_max - (x, y) maximum velocity in x and y direction a robot can take
# v_pref - (x, y) preferred velocity of the robot
################################################################################
    
class Bot(object):
    
    def __init__(self, pos, vel, rad, v_max, v_pref):
     
        self.position = array(pos)
        self.velocity = array(vel)
        self.radius = rad
        self.max_speed = v_max
        self.pref_velocity = array(v_pref)

# Class created in order to throw infeasible error
class InfeasibleError(RuntimeError):
    pass

##########################################################################################
# Function Name : optimize_hp
# Arguments : lines(list of the lines), optimalPoint(initial most basic optimal point
# through linear programming)
# Return : Returns the most optimal point that exists 
##########################################################################################

def optimize_hp(lines, optimalPoint):

    point = optimalPoint
    for i, line in enumerate(lines):
        # If current point already exists on the half-plane
        if dot(point - line.pt, line.dir) >= 0:
            continue

        # Otherwise, the new optimum must lie on the newly added line. Compute
        # the feasible interval of the intersection of all the lines added so
        # far with the current one.
        prevLines = itertools.islice(lines, i)
        leftDist, rightDist = line_halfplane_intersect(line, prevLines)

        # Now project the optimal point onto the line segment defined by the
        # the above bounds. This gives us our new best point.
        point = point_line_project(line, optimalPoint, leftDist, rightDist)
        
    return point

##########################################################################################
# Function Name : point_line_project
# Arguments : line, point, leftBound, rightBound
# Return : Returns a point projected by the line
##########################################################################################

def point_line_project(line, point, leftBound, rightBound):

    # print("left_bound=%s, right_bound=%s" % (left_bound, right_bound))
    newDirection = perpendicular(line.dir)
    # print("new_dir=%s" % new_dir)
    projLength = dot(point - line.pt, newDirection)
    # print("proj_len=%s" % proj_len)
    clampedLength = clip(projLength, leftBound, rightBound)
    # print("clamped_len=%s" % clamped_len)
    return line.pt + newDirection * clampedLength

##########################################################################################
# Function Name : line_halfplane_intersect
# Arguments :line (OCRA line of the current robot), otherLines (OCRA lines for the other 
# robots)
# Return : Solution of the intersection between the lines 
# Algorithm : Uses linear programming to solve the half-plane equations
##########################################################################################

def line_halfplane_intersect(line, otherLines):

    #"Left" is the negative of the canonical direction of the line.
    # "Right" is positive.
    leftDistance = float("-inf")
    rightDistance = float("inf")
    for prevLine in otherLines:
        num1 = dot(prevLine.dir, line.pt - prevLine.pt)
        den1 = det((line.dir, prevLine.dir))
        
        num = num1
        den = den1

        # Check for zero denominator, since ZeroDivisionError (or rather
        # FloatingPointError) won't necessarily be raised if using numpy.
        if den == 0:
            # If half-planes are parallel.
            if num < 0:
                # The intersection of the half-planes is empty; there is no
                # solution.
                raise InfeasibleError
            else:
                # The *half-planes* intersect, but their lines don't cross, so
                # ignore.
                continue

        # Signed offset of the point of intersection, relative to the line's
        # anchor point, in units of the line's direction.
        offset = num / den
        if den > 0:
            # Point of intersection is to the right.
            rightDistance = min((rightDistance, offset))
        else:
            # Point of intersection is to the left.
            leftDistance = max((leftDistance, offset))

        if leftDistance > rightDistance:
            # The interval is inconsistent, so the feasible region is empty.
            raise InfeasibleError
    return leftDistance, rightDistance

def perpendicular(a):
    return array((a[1], -a[0]))

def norm_sq(x):
    return dot(x, x)

def norm(x):
    return sqrt(norm_sq(x))

def normalized(x):
    l = norm_sq(x)
    assert l > 0, (x, l)
    return x / sqrt(l)

##########################################################################################
# Function Name : orca
# Arguments : bot(robot for which we are calculating ORCA), colliding_bots(other robots in 
# the area) , t(current time), dt(time for which the ORCA calculation is needed to be done)
# Return : point of the intersection of the OCRA lines
# Algorithm : Uses above linear programming functions to compute the intersection of the
# ORCA lines
##########################################################################################
     
def orca(bot, colliding_bots, t, dt):
 
    l = list()
    for c in colliding_bots:
        dv, n = ca_velocity(bot, c, t, dt)
        line = Line(bot.velocity + dv / 2, n)
        l.append(line)
    return optimize_hp(l, bot.pref_velocity), l

##########################################################################################
# Function Name : ca_velocity
# Arguments : bot(robot for which we are calculating ORCA), colliding_bots(other robots in 
# the area) , t(current time), dt(time for which the ORCA calculation is needed to be done)
# Return : velocity which can avoid the collision with other robots
##########################################################################################

def ca_velocity(bot, colliding_bot, t, dt):

    x = -(bot.position - colliding_bot.position)
    v = bot.velocity - colliding_bot.velocity
    r = bot.radius + colliding_bot.radius

    x_len_sq = norm_sq(x)
    # Truncating the cone of the velocity obstacle
    if x_len_sq >= r * r: 
        
        center_new = x/t * (1 - (r*r)/x_len_sq)
        # If the velocity is at the front of the cone
        if dot(v - center_new, center_new) < 0:
            w = v - x/t
            u = normalized(w) * r/t - w
            n = normalized(w)
        # If the velocity is not at the front of the cone i.e. at any other point on the cone
        else: 
            leg_len = sqrt(x_len_sq - r*r)
           
            sine = copysign(r, det((v, x)))
            rot = array(((leg_len, sine),(-sine, leg_len)))
            rotated_x = rot.dot(x) / x_len_sq
            n = perpendicular(rotated_x)
            
            if sine < 0:
                n = -n
            u = rotated_x * dot(v, rotated_x) - v
    else: # If the lines are already intersecting
        w = v - x/dt
        u = normalized(w) * r/dt - w
        n = normalized(w)
    
    return u, n


speed = 10
robots = []


# initializing the robot's position, velocity, radius, preferred velocity
# uncomment
robots.append(Bot((-20.,0.), (10., 5.), 3., speed, (5., 0.))) # Robot 1
robots.append(Bot((20.,0.), (-10., 5.), 3., speed, (-5., 0.))) # Robot 2
#robots.append(Bot((0.,-20.), (-10., 5.), 3., speed, (0., 5.))) # Robot 3
#robots.append(Bot((0.,20.), (10., 5.), 3., speed, (0., -5.))) # Robot 4

# defining the colors for various robots
colors = [(255, 0, 0),(0, 255, 0),(0, 0, 255),(255, 255, 0),(0, 255, 255),(255, 0, 255)]

pygame.init()

dim = (1280, 720)
screen = pygame.display.set_mode(dim)

O = array(dim) / 2  # Screen position of origin.
scale = 15  # Drawing scale.

clock = pygame.time.Clock()
FPS = 30
dt = 1/FPS
tau = 2

def drawRobot(robot, color):
    pygame.draw.circle(screen, color, rint(robot.position * scale + O).astype(int), int(round(robot.radius * scale)), 0)

def drawOrcaPredictionCircle(a, b):
    for x in linspace(0, tau, 21):
        if x == 0:
            continue
        pygame.draw.circle(screen, pygame.Color(255, 255, 255), rint((-(a.position - b.position) / x + a.position) * scale + O).astype(int), int(round((a.radius + b.radius) * scale / x)), 1)

def drawVelocity(a, color):
    pygame.draw.line(screen, color, rint(a.position * scale + O).astype(int), rint((a.position + a.velocity) * scale + O).astype(int), 1)

run_until = True
clock_ticks = 0
orca_lines = [[]] * len(robots)
while run_until:
    clock_ticks += clock.tick(FPS)

    while clock_ticks >= dt * 1000:
        clock_ticks -= dt * 1000
        
        new_vels = [None] * len(robots)
        for i, robot in enumerate(robots):
            candidates = robots[:i] + robots[i + 1:]

            new_vels[i], orca_lines[i] = orca(robot, candidates, tau, dt)
        

        for i, robot in enumerate(robots):
            robot.velocity = new_vels[i]
            robot.position += robot.velocity * dt

    screen.fill(pygame.Color(80, 80, 80))
    
    for robot in robots[1:]:
        drawOrcaPredictionCircle(robots[0], robot)
    
    for robot, color in zip(robots, itertools.cycle(colors)):
        drawRobot(robot, color)
        drawVelocity(robot, color)
        
    
    for line in orca_lines[0]:
        
        # Draw ORCA line
        alpha = robots[0].position + line.pt + perpendicular(line.dir) * 100
        beta = robots[0].position + line.pt + perpendicular(line.dir) * -100
        pygame.draw.line(screen, (255, 255, 255), rint(alpha * scale + O).astype(int), rint(beta * scale + O).astype(int), 1)

        # Draw normal to ORCA line
        gamma = robots[0].position + line.pt
        delta = robots[0].position + line.pt + line.dir
    
    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run_until = False
pygame.quit()


# In[ ]:




