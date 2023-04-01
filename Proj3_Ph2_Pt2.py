# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import cv2
import numpy as np
import heapq
import math
import time


def zero_rpm1(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    new_x = parent_node[0]
    new_y = parent_node[1]
    new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    cost = 0
    UL = 0
    UR = rpm1
    obs_match = False
    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        cost = cost + (delta_x ** 2 + delta_y ** 2) ** (0.5)
        new_x = new_x + delta_x
        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta
        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True
            break
    new_theta = new_theta * 180 / math.pi
    if new_theta >= 360:
        new_theta = new_theta - 360 * int(new_theta / 360)
    elif new_theta < 0:
        new_theta = new_theta - 360 * int(new_theta / 360 - 1)
    if round(new_theta) == 360:
        new_theta = 0
    if obs_match == True:
        new_node = ((-1000, -1000, -1000), -1)
    else:
        new_node = ((new_x, new_y, new_theta), cost)
    return new_node


def rpm1_zero(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    new_x = parent_node[0]
    new_y = parent_node[1]
    new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    cost = 0
    UL = rpm1
    UR = 0
    obs_match = False
    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        cost = cost + (delta_x ** 2 + delta_y ** 2) ** (0.5)
        new_x = new_x + delta_x
        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta
        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True
            break
    new_theta = new_theta * 180 / math.pi

    if new_theta >= 360:
        new_theta = new_theta - 360 * int(new_theta / 360)
    elif new_theta < 0:
        new_theta = new_theta - 360 * int(new_theta / 360 - 1)
    if round(new_theta) == 360:
        new_theta = 0
    if obs_match == True:
        new_node = ((-1000, -1000, -1000), -1)
        # parent_rounded_x = int(round(parent_node[0] * 2))
        # parent_rounded_y = int(round(parent_node[1] * 2))
        # theta_index = int(parent_node[2] / 30)
        # visited_nodes[parent_rounded_y][parent_rounded_x][theta_index] = 1
    else:
        new_node = ((new_x, new_y, new_theta), cost)
    return new_node


def rpm1_rpm1(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    new_x = parent_node[0]
    new_y = parent_node[1]
    new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    cost = 0
    UL = rpm1
    UR = rpm1
    obs_match = False
    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        cost = cost + (delta_x ** 2 + delta_y ** 2) ** (0.5)
        new_x = new_x + delta_x

        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta
        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True
            break
    new_theta = new_theta * 180 / math.pi
    if new_theta >= 360:
        new_theta = new_theta - 360 * int(new_theta / 360)
    elif new_theta < 0:
        new_theta = new_theta - 360 * int(new_theta / 360 - 1)
    if round(new_theta) == 360:
        new_theta = 0
    if obs_match == True:
        new_node = ((-1000, -1000, -1000), -1)
        # parent_rounded_x = int(round(parent_node[0] * 2))
        # parent_rounded_y = int(round(parent_node[1] * 2))
        # theta_index = int(parent_node[2] / 30)
        # visited_nodes[parent_rounded_y][parent_rounded_x][theta_index] = 1
    else:
        new_node = ((new_x, new_y, new_theta), cost)
    return new_node


def zero_rpm2(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    new_x = parent_node[0]
    new_y = parent_node[1]
    new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    cost = 0
    UL = 0
    UR = rpm2
    obs_match = False
    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        cost = cost + (delta_x ** 2 + delta_y ** 2) ** (0.5)
        new_x = new_x + delta_x
        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta
        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True
            break
    new_theta = new_theta * 180 / math.pi
    if new_theta >= 360:
        new_theta = new_theta - 360 * int(new_theta / 360)
    elif new_theta < 0:
        new_theta = new_theta - 360 * int(new_theta / 360 - 1)

    if round(new_theta) == 360:
        new_theta = 0
    if obs_match == True:
        new_node = ((-1000, -1000, -1000), -1)
        # parent_rounded_x = int(round(parent_node[0] * 2))
        # parent_rounded_y = int(round(parent_node[1] * 2))
        # theta_index = int(parent_node[2] / 30)
        # visited_nodes[parent_rounded_y][parent_rounded_x][theta_index] = 1
    else:
        new_node = ((new_x, new_y, new_theta), cost)
    return new_node


def rpm2_zero(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    new_x = parent_node[0]
    new_y = parent_node[1]
    new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    cost = 0
    UL = rpm2
    UR = 0
    obs_match = False
    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        cost = cost + (delta_x ** 2 + delta_y ** 2) ** (0.5)
        new_x = new_x + delta_x
        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta
        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True
            break
    new_theta = new_theta * 180 / math.pi
    if new_theta >= 360:
        new_theta = new_theta - 360 * int(new_theta / 360)
    elif new_theta < 0:
        new_theta = new_theta - 360 * int(new_theta / 360 - 1)

    if round(new_theta) == 360:
        new_theta = 0
    if obs_match == True:
        new_node = ((-1000, -1000, -1000), -1)
        # parent_rounded_x = int(round(parent_node[0] * 2))
        # parent_rounded_y = int(round(parent_node[1] * 2))
        # theta_index = int(parent_node[2] / 30)
        # visited_nodes[parent_rounded_y][parent_rounded_x][theta_index] = 1
    else:
        new_node = ((new_x, new_y, new_theta), cost)
    return new_node


def rpm2_rpm2(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    new_x = parent_node[0]
    new_y = parent_node[1]
    new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    cost = 0
    UL = rpm2
    UR = rpm2
    obs_match = False
    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        cost = cost + (delta_x ** 2 + delta_y ** 2) ** (0.5)
        new_x = new_x + delta_x
        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta
        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True
            break
    new_theta = new_theta * 180 / math.pi
    if new_theta >= 360:
        new_theta = new_theta - 360 * int(new_theta / 360)
    elif new_theta < 0:
        new_theta = new_theta - 360 * int(new_theta / 360 - 1)

    if round(new_theta) == 360:
        new_theta = 0
    if obs_match == True:
        new_node = ((-1000, -1000, -1000), -1)
        # parent_rounded_x = int(round(parent_node[0] * 2))
        # parent_rounded_y = int(round(parent_node[1] * 2))
        # theta_index = int(parent_node[2] / 30)
        # visited_nodes[parent_rounded_y][parent_rounded_x][theta_index] = 1
    else:
        new_node = ((new_x, new_y, new_theta), cost)
    return new_node


def rpm1_rpm2(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    new_x = parent_node[0]
    new_y = parent_node[1]
    new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    cost = 0
    UL = rpm1
    UR = rpm2
    obs_match = False
    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        cost = cost + (delta_x ** 2 + delta_y ** 2) ** (0.5)
        new_x = new_x + delta_x
        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta
        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True
            break
    new_theta = new_theta * 180 / math.pi
    if new_theta >= 360:
        new_theta = new_theta - 360 * int(new_theta / 360)
    elif new_theta < 0:
        new_theta = new_theta - 360 * int(new_theta / 360 - 1)
    if round(new_theta) == 360:
        new_theta = 0
    if obs_match == True:
        new_node = ((-1000, -1000, -1000), -1)
        # parent_rounded_x = int(round(parent_node[0] * 2))
        # parent_rounded_y = int(round(parent_node[1] * 2))
        # theta_index = int(parent_node[2] / 30)
        # visited_nodes[parent_rounded_y][parent_rounded_x][theta_index] = 1
    else:
        new_node = ((new_x, new_y, new_theta), cost)
    return new_node


def rpm2_rpm1(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    new_x = parent_node[0]
    new_y = parent_node[1]
    new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    cost = 0
    UL = rpm2
    UR = rpm1
    obs_match = False
    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        cost = cost + (delta_x ** 2 + delta_y ** 2) ** (0.5)
        new_x = new_x + delta_x
        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta
        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True
            break
    new_theta = new_theta * 180 / math.pi
    if new_theta >= 360:
        new_theta = new_theta - 360 * int(new_theta / 360)

    elif new_theta < 0:
        new_theta = new_theta - 360 * int(new_theta / 360 - 1)
    if round(new_theta) == 360:
        new_theta = 0
    if obs_match == True:
        new_node = ((-1000, -1000, -1000), -1)
        # parent_rounded_x = int(round(parent_node[0] * 2))
        # parent_rounded_y = int(round(parent_node[1] * 2))
        # theta_index = int(parent_node[2] / 30)
        # visited_nodes[parent_rounded_y][parent_rounded_x][theta_index] = 1
    else:
        new_node = ((new_x, new_y, new_theta), cost)
    return new_node


def print_zero_rpm1(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    obs_match = False
    if forward == False:
        new_x = coord[0]
        new_y = coord[1]
        new_theta = coord[2] * math.pi / 180
    else:
        parent_node = cost_to_come[coord]['parent node']
        new_x = parent_node[0]
        new_y = parent_node[1]
        new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    UL = 0
    UR = rpm1

    # x1=2*int(round(node[0]))
    # y1=500-(int(round(node[1])))*2

    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        new_x = new_x + delta_x
        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta

        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True

    if obs_match == False:
        if forward == False:
            new_x = coord[0]
            new_y = coord[1]
            new_theta = coord[2] * math.pi / 180
        else:
            parent_node = cost_to_come[coord]['parent node']
            new_x = parent_node[0]
            new_y = parent_node[1]
            new_theta = parent_node[2] * math.pi / 180
        t = 0
        while t < move_time:
            t = t + dt
            delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
            delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
            delta_theta = (r / L) * (UR - UL) * dt
            pt1 = ((int(round(new_x + 50 ))), (200 - int(round(new_y + 100))))
            pt2 = ((int(round(new_x + 50 + delta_x))), (200 - int(round(new_y + 100 + delta_y))))
            if forward == False:
                cv2.line(visual_map_explore, pt1, pt2, (255, 0, 0))
            else:
                cv2.line(visual_map_explore, pt1, pt2, (0, 255, 0))
            new_x = new_x + delta_x
            new_y = new_y + delta_y
            new_theta = new_theta + delta_theta


def print_rpm1_zero(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    obs_match = False
    if forward == False:
        new_x = coord[0]
        new_y = coord[1]
        new_theta = coord[2] * math.pi / 180
    else:
        parent_node = cost_to_come[coord]['parent node']
        new_x = parent_node[0]
        new_y = parent_node[1]
        new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    cost = 0
    UL = rpm1
    UR = 0

    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        new_x = new_x + delta_x
        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta

        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True

    if obs_match == False:
        if forward == False:
            new_x = coord[0]
            new_y = coord[1]
            new_theta = coord[2] * math.pi / 180
        else:
            parent_node = cost_to_come[coord]['parent node']
            new_x = parent_node[0]
            new_y = parent_node[1]
            new_theta = parent_node[2] * math.pi / 180
        t = 0
        while t < move_time:
            t = t + dt
            delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
            delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
            delta_theta = (r / L) * (UR - UL) * dt
            pt1 = ((int(round(new_x + 50 ))), (200 - int(round(new_y + 100))))
            pt2 = ((int(round(new_x + 50 + delta_x))), (200 - int(round(new_y + 100 + delta_y))))
            if forward == False:
                cv2.line(visual_map_explore, pt1, pt2, (255, 0, 0))
            else:
                cv2.line(visual_map_explore, pt1, pt2, (0, 255, 0))
            new_x = new_x + delta_x
            new_y = new_y + delta_y
            new_theta = new_theta + delta_theta


def print_rpm1_rpm1(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    obs_match = False
    if forward == False:
        new_x = coord[0]
        new_y = coord[1]
        new_theta = coord[2] * math.pi / 180
    else:
        parent_node = cost_to_come[coord]['parent node']
        new_x = parent_node[0]
        new_y = parent_node[1]
        new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    cost = 0
    UL = rpm1
    UR = rpm1

    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        new_x = new_x + delta_x
        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta

        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True

    if obs_match == False:
        if forward == False:
            new_x = coord[0]
            new_y = coord[1]
            new_theta = coord[2] * math.pi / 180
        else:
            parent_node = cost_to_come[coord]['parent node']
            new_x = parent_node[0]
            new_y = parent_node[1]
            new_theta = parent_node[2] * math.pi / 180
        t = 0
        while t < move_time:
            t = t + dt
            delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
            delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
            delta_theta = (r / L) * (UR - UL) * dt
            pt1 = ((int(round(new_x + 50 ))), (200 - int(round(new_y + 100))))
            pt2 = ((int(round(new_x + 50 + delta_x))), (200 - int(round(new_y + 100 + delta_y))))
            if forward == False:
                cv2.line(visual_map_explore, pt1, pt2, (255, 0, 0))
            else:
                cv2.line(visual_map_explore, pt1, pt2, (0, 255, 0))
            new_x = new_x + delta_x
            new_y = new_y + delta_y
            new_theta = new_theta + delta_theta


def print_zero_rpm2(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    obs_match = False
    if forward == False:
        new_x = coord[0]
        new_y = coord[1]
        new_theta = coord[2] * math.pi / 180
    else:
        parent_node = cost_to_come[coord]['parent node']
        new_x = parent_node[0]
        new_y = parent_node[1]
        new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    cost = 0
    UL = 0
    UR = rpm2

    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        new_x = new_x + delta_x
        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta

        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True

    if obs_match == False:
        if forward == False:
            new_x = coord[0]
            new_y = coord[1]
            new_theta = coord[2] * math.pi / 180
        else:
            parent_node = cost_to_come[coord]['parent node']
            new_x = parent_node[0]
            new_y = parent_node[1]
            new_theta = parent_node[2] * math.pi / 180
        t = 0
        while t < move_time:
            t = t + dt
            delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
            delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
            delta_theta = (r / L) * (UR - UL) * dt
            pt1 = ((int(round(new_x + 50 ))), (200 - int(round(new_y + 100))))
            pt2 = ((int(round(new_x + 50 + delta_x))), (200 - int(round(new_y + 100 + delta_y))))
            if forward == False:
                cv2.line(visual_map_explore, pt1, pt2, (255, 0, 0))
            else:
                cv2.line(visual_map_explore, pt1, pt2, (0, 255, 0))
            new_x = new_x + delta_x
            new_y = new_y + delta_y
            new_theta = new_theta + delta_theta


def print_rpm2_zero(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    obs_match = False
    if forward == False:
        new_x = coord[0]
        new_y = coord[1]
        new_theta = coord[2] * math.pi / 180
    else:
        parent_node = cost_to_come[coord]['parent node']
        new_x = parent_node[0]
        new_y = parent_node[1]
        new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    cost = 0
    UL = rpm2
    UR = 0

    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        new_x = new_x + delta_x
        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta

        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True

    if obs_match == False:
        if forward == False:
            new_x = coord[0]
            new_y = coord[1]
            new_theta = coord[2] * math.pi / 180
        else:
            parent_node = cost_to_come[coord]['parent node']
            new_x = parent_node[0]
            new_y = parent_node[1]
            new_theta = parent_node[2] * math.pi / 180
        t = 0
        while t < move_time:
            t = t + dt
            delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
            delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
            delta_theta = (r / L) * (UR - UL) * dt
            pt1 = ((int(round(new_x + 50 ))), (200 - int(round(new_y + 100))))
            pt2 = ((int(round(new_x + 50 + delta_x))), (200 - int(round(new_y + 100 + delta_y))))
            if forward == False:
                cv2.line(visual_map_explore, pt1, pt2, (255, 0, 0))
            else:
                cv2.line(visual_map_explore, pt1, pt2, (0, 255, 0))
            new_x = new_x + delta_x
            new_y = new_y + delta_y
            new_theta = new_theta + delta_theta


def print_rpm2_rpm2(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    obs_match = False
    if forward == False:
        new_x = coord[0]
        new_y = coord[1]
        new_theta = coord[2] * math.pi / 180
    else:
        parent_node = cost_to_come[coord]['parent node']
        new_x = parent_node[0]
        new_y = parent_node[1]
        new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    cost = 0
    UL = rpm2
    UR = rpm2

    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        new_x = new_x + delta_x
        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta

        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True

    if obs_match == False:
        if forward == False:
            new_x = coord[0]
            new_y = coord[1]
            new_theta = coord[2] * math.pi / 180
        else:
            parent_node = cost_to_come[coord]['parent node']
            new_x = parent_node[0]
            new_y = parent_node[1]
            new_theta = parent_node[2] * math.pi / 180
        t = 0
        while t < move_time:
            t = t + dt
            delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
            delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
            delta_theta = (r / L) * (UR - UL) * dt
            pt1 = ((int(round(new_x + 50 ))), (200 - int(round(new_y + 100))))
            pt2 = ((int(round(new_x + 50 + delta_x))), (200 - int(round(new_y + 100 + delta_y))))
            if forward == False:
                cv2.line(visual_map_explore, pt1, pt2, (255, 0, 0))
            else:
                cv2.line(visual_map_explore, pt1, pt2, (0, 255, 0))
            new_x = new_x + delta_x
            new_y = new_y + delta_y
            new_theta = new_theta + delta_theta


def print_rpm1_rpm2(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    obs_match = False
    if forward == False:
        new_x = coord[0]
        new_y = coord[1]
        new_theta = coord[2] * math.pi / 180
    else:
        parent_node = cost_to_come[coord]['parent node']
        new_x = parent_node[0]
        new_y = parent_node[1]
        new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    cost = 0
    UL = rpm1
    UR = rpm2

    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        new_x = new_x + delta_x
        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta

        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True

    if obs_match == False:
        if forward == False:
            new_x = coord[0]
            new_y = coord[1]
            new_theta = coord[2] * math.pi / 180
        else:
            parent_node = cost_to_come[coord]['parent node']
            new_x = parent_node[0]
            new_y = parent_node[1]
            new_theta = parent_node[2] * math.pi / 180
        t = 0
        while t < move_time:
            t = t + dt
            delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
            delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
            delta_theta = (r / L) * (UR - UL) * dt
            pt1 = ((int(round(new_x + 50 ))), (200 - int(round(new_y + 100))))
            pt2 = ((int(round(new_x + 50 + delta_x))), (200 - int(round(new_y + 100 + delta_y))))
            if forward == False:
                cv2.line(visual_map_explore, pt1, pt2, (255, 0, 0))
            else:
                cv2.line(visual_map_explore, pt1, pt2, (0, 255, 0))
            new_x = new_x + delta_x
            new_y = new_y + delta_y
            new_theta = new_theta + delta_theta


def print_rpm2_rpm1(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2):
    obs_match = False
    if forward == False:
        new_x = coord[0]
        new_y = coord[1]
        new_theta = coord[2] * math.pi / 180
    else:
        parent_node = cost_to_come[coord]['parent node']
        new_x = parent_node[0]
        new_y = parent_node[1]
        new_theta = parent_node[2] * math.pi / 180
    t = 0
    r = wheel_radius
    L = wheel_dist
    dt = move_time / 10
    cost = 0
    UL = rpm2
    UR = rpm1

    while t < move_time:
        t = t + dt
        delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
        delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
        delta_theta = (r / L) * (UR - UL) * dt
        new_x = new_x + delta_x
        new_y = new_y + delta_y
        new_theta = new_theta + delta_theta

        rounded_x = round(new_x)
        rounded_y = round(new_y)
        new_coord = (rounded_x, rounded_y)
        if new_coord in obstacle_list:
            obs_match = True

    if obs_match == False:
        if forward == False:
            new_x = coord[0]
            new_y = coord[1]
            new_theta = coord[2] * math.pi / 180
        else:
            parent_node = cost_to_come[coord]['parent node']
            new_x = parent_node[0]
            new_y = parent_node[1]
            new_theta = parent_node[2] * math.pi / 180
        t = 0
        while t < move_time:
            t = t + dt
            delta_x = 0.5 * r * (UL + UR) * round(math.cos(new_theta), 5) * dt
            delta_y = 0.5 * r * (UL + UR) * round(math.sin(new_theta), 5) * dt
            delta_theta = (r / L) * (UR - UL) * dt
            pt1 = ((int(round(new_x + 50 ))), (200 - int(round(new_y + 100))))
            pt2 = ((int(round(new_x + 50 + delta_x))), (200 - int(round(new_y + 100 + delta_y))))
            if forward == False:
                cv2.line(visual_map_explore, pt1, pt2, (255, 0, 0))
            else:
                cv2.line(visual_map_explore, pt1, pt2, (0, 255, 0))
            new_x = new_x + delta_x
            new_y = new_y + delta_y
            new_theta = new_theta + delta_theta


def generate_path(reverse_path):
    next_node = []
    while next_node != "N/A":
        search_for = reverse_path[-1]
        reverse_path.append((cost_to_come[search_for]['parent node']))
        next_node = cost_to_come[search_for]['parent node']

    print("This is the reverse path from goal to start", reverse_path)

    # This loop creates the forward path to goal
    t = 0
    forward_path = []
    for t in range(len(reverse_path)):
        forward_path.append(reverse_path.pop(-1))

    # # this eliminates the start node from forward_path
    forward_path.pop(0)
    return forward_path


def check_for_goal(parent_node, goal_node):
    SolutionFound = False
    x = parent_node[0]
    y = parent_node[1]
    goal_x = goal_node[0]
    goal_y = goal_node[1]
    if (((goal_x - x) ** 2 + (goal_y - y) ** 2) ** (0.5)) <= ((wheel_radius * 2 * math.pi)):
        SolutionFound = True
    return SolutionFound


def obstacle_check(parent_node, clearance, obstacle_match):  # used to confirm user inputs
    obstacle_match = True
    x = parent_node[0]
    y = parent_node[1]
    if (x >= (100-(clearance))) and (x <= (115+clearance)) and (y >= -25-clearance) and (y <= (100)):  # Obstacle A check
        obstacle_match=True
    else:
        if (x >= (200-clearance)) and (x <= (215+clearance)) and (y >= (-100)) and (y <= 25+clearance):  # Obstacle B check
            obstacle_match = True
        else:

            # Obstacle C1 check
            if ((x - 350)**2 + (y - 10)**2) <= (50+clearance)**2:
                obstacle_match = True
            else:
                obstacle_match = False
    return obstacle_match


def nodes_to_plot(coord):
    parent_node = coord

    left_pivot_rpm1 = zero_rpm1(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    right_pivot_rpm1 = rpm1_zero(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    straight_rpm1 = rpm1_rpm1(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)

    left_pivot_rpm2 = zero_rpm2(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    right_pivot_rpm2 = rpm2_zero(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    straight_rpm2 = rpm2_rpm2(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    turn1 = rpm1_rpm2(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    turn2 = rpm2_rpm1(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)

    action_dict = {0: left_pivot_rpm1, 1: right_pivot_rpm1, 2: straight_rpm1, 3: left_pivot_rpm2, 4: right_pivot_rpm2,
                   5: straight_rpm2, 6: turn1, 7: turn2}

    for j in range(len(action_dict)):

        # Setting up Rounded node to determine if node has been found
        action_node = action_dict[j][0]
        # print("action node", action_node)
        action_x = action_node[0]
        action_y = action_node[1]
        # if statement checks if new node is outside map (can happen with larger step sizes)
        if action_x >= 600 or action_x <= -50 or action_y >= 100 or action_y <= -100:
            action_node = (-1000, -1000, -1000)
        action_theta = action_node[2]
        rounded_x = round(action_x)
        rounded_y = round(action_y)
        action_coord = (rounded_x, rounded_y)
        # print("action coord", action_coord)
        theta_index = int(action_theta / 30)
        if action_coord in obstacle_list:  # obs_check==True:
            match = True
        else:
            if j == 0:
                print_zero_rpm1(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
            elif j == 1:
                print_rpm1_zero(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
            elif j == 2:
                print_rpm1_rpm1(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
            elif j == 3:
                print_zero_rpm2(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
            elif j == 4:
                print_rpm2_zero(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
            elif j == 5:
                print_rpm2_rpm2(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
            elif j == 6:
                print_rpm1_rpm2(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
            else:
                print_rpm2_rpm1(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)


def forward_path_plot(coord):
    parent_node = coord
    move = cost_to_come[parent_node]['previous move']

    if move == 0:
        print_zero_rpm1(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    elif move == 1:
        print_rpm1_zero(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    elif move == 2:
        print_rpm1_rpm1(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    elif move == 3:
        print_zero_rpm2(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    elif move == 4:
        print_rpm2_zero(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    elif move == 5:
        print_rpm2_rpm2(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    elif move == 6:
        print_rpm1_rpm2(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    else:
        print_rpm2_rpm1(coord, wheel_radius, wheel_dist, move_time, rpm1, rpm2)


do_nothing = False

# Initialize the starting robot parameters

clearance = 0
start_cost = 0.0
wheel_radius = 0.033*100
robot_radius = 0.105*100
wheel_dist = 0.16*100
move_time = 1

theta_choices = set([330, 300, 270, 240, 210, 180, 150, 120, 90, 60, 30, 0])
safety_zone = float(input('What is the minimum distance you want the robot to avoid obstacles by?\n'))
# step_size=float(input('How far does the robot move each action?\n'))
rpm1 = float(input('What is the first RPM setting of the robot?\n'))
rpm2 = float(input('What is the second RPM setting of the robot?\n'))

clearance = safety_zone + robot_radius
print("clearance", +clearance)

# This part of the code builds the map and obstacles based on inputed robot size
visual_map = np.zeros((200, 600, 3), np.uint8)
visual_map[0:200, 0:600, :] = [0, 0, 255]
visited_nodes = np.zeros((200, 600, 12))

cost_to_come = []
x = 0
y = 0
node = 0
cost_to_come = {}
obstacles = []
# Loop builds obstacle list and primes np array for visualization of the space
for i in range(600):
    for j in range(200):
        x = i - 50
        y = 200 - j - 100

        if (x >= (100-(clearance))) and (x <= (115+clearance)) and (y >= -25-clearance) and (y <= (100)):  # Obstacle A check
            visited_nodes[j][i][:] = -1.0
            obstacles.append((x,y))
        else:
            if (x >= (200-clearance)) and (x <= (215+clearance)) and (y >= (-100)) and (y <= 25+clearance):  # Obstacle B check
                visited_nodes[j][i][:] = -1.0
                obstacles.append((x, y))
            else:
                # Obstacle C1 check
                if ((x - 350)**2 + (y - 10)**2) <= (50+clearance)**2:
                    visited_nodes[j][i][:] = -1.0
                    obstacles.append((x, y))
                else:
                    # Walls check
                    if (x <= -50 + clearance) or (x >= (600 - clearance)) or (y <= -100+clearance) or (
                            y >= (100 - clearance)):
                        visited_nodes[j][i][:] = -1.0
                        obstacles.append((x, y))
                    else:
                        # visited_nodes[j][i][:] = 0
                        # cost_to_come[node]={'x': x,'y': y,'parent node': "N/A", 'cost to come': float('inf')}
                        visual_map[j, i, :] = [100, 100, 100]

# obstacle_list used when new nodes are created to check if they are in obstacle space
obstacle_list = set(obstacles)

# Visualize Space
cv2.imshow("Zeros matx", visual_map)  # show numpy array
cv2.waitKey(0)  # wait for ay key to exit window
cv2.destroyAllWindows()  # close all windows

# Define start node by x,y, and theta coordinates
bad_choice = True
obstacle_match = True
while bad_choice == True:
    start_x = float(input('What is the x coordinate in meters (positive or negative, 1 decimal place max) of your starting point?\n'))
    start_x = start_x*100
    start_y = float(input('What is the y coordinate in meters (positive or negative, 1 decimal place max) of your starting point?\n'))
    start_y = start_y *100
    start_theta = input(
        'What is the initial facing of the robot?  Enter a positive or negative multiple 30 between 0 and 330.\n')
    start_theta = int(start_theta)
    start_node = (start_x, start_y, start_theta)
    bad_choice_check = obstacle_check(start_node, clearance, obstacle_match)
    if bad_choice_check == True:
        print('This is an invalid starting position, please try again.\n')
    else:
        if abs(start_theta) in theta_choices:
            bad_choice = False
        else:
            print('This is an invalid initial facing of the robot, please try again.\n')

start_theta_index = int(start_theta / 30)
visited_nodes[int((start_y+1))][int((start_x+0.5))][start_theta_index] = 0

# Define goal node by x,y, and theta coordinates
bad_choice = True
obstacle_match = True
while bad_choice == True:
    goal_x = float(input('What is the x coordinate in meters (positive or negative, 1 decimal place max) of your target position?\n'))
    goal_x = goal_x *100
    goal_y = float(input('What is the y coordinate in meters (positive or negative, 1 decimal place max) of your target position?\n'))
    goal_y = goal_y *100
    goal_node = (goal_x, goal_y)
    bad_choice_check = obstacle_check(goal_node, clearance, obstacle_match)
    if bad_choice_check == True:
        print('This is an invalid goal position, please try again.\n')
    else:
        bad_choice = False

start = time.time()
# Initializing node map
distance_to_goal = ((goal_x - start_x) ** 2 + (goal_y - start_y) ** 2) ** (0.5)
cost_to_come[start_node] = {'x': start_x, 'y': start_y, 'theta': start_theta, 'parent node': "N/A", 'cost to come': 0,
                            'total cost': distance_to_goal, 'previous move': "NA"}

SolutionFound = False
counter = 0

# Initialize list of nodes that need to be expanded/investigated/have moves applied
queue = []
queue = [[cost_to_come[start_node]['total cost'], start_node, "NA"]]

closed_list = []
closed_list_check = set()
# closed_list_with_move = []

while SolutionFound != True:  # counter<100: #

    heapq.heapify(queue)

    # identify the parent node and eliminate it from the list of nodes that need to be investigated
    parent_node = heapq.heappop(queue)

    parent_node = parent_node[1]
    parent_node_cost = cost_to_come[parent_node]['cost to come']
    # check if the last popped node is a match for goal
    # if it's a match, initializes reverse_path list and adds node to node map

    SolutionFound = check_for_goal(parent_node, goal_node)
    if SolutionFound == True:
        print(cost_to_come[parent_node])
        reverse_path = [goal_node, parent_node]
        print(reverse_path)
        break

    closed_list.append(parent_node)
    closed_list_check.add(parent_node)

    # #     #perform "moves" on "parent" node to create "new" nodes

    left_pivot_rpm1 = zero_rpm1(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    right_pivot_rpm1 = rpm1_zero(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    straight_rpm1 = rpm1_rpm1(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    left_pivot_rpm2 = zero_rpm2(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    right_pivot_rpm2 = rpm2_zero(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    straight_rpm2 = rpm2_rpm2(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    turn1 = rpm1_rpm2(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)
    turn2 = rpm2_rpm1(parent_node, wheel_radius, wheel_dist, move_time, rpm1, rpm2)

    # stores the "new" nodes in another dictionary for future use in loop
    action_dict = {0: left_pivot_rpm1, 1: right_pivot_rpm1, 2: straight_rpm1, 3: left_pivot_rpm2, 4: right_pivot_rpm2,
                   5: straight_rpm2, 6: turn1, 7: turn2}

    # initialize variable before loop begins
    match = False

    # for each action in action_dict loop runs
    for j in range(len(action_dict)):
        match = False

        # Setting up Rounded node to determine if node has been found
        action_node = action_dict[j][0]
        action_x = action_node[0]
        action_y = action_node[1]
        # if statement checks if new node is outside map (can happen with larger step sizes)
        if action_x >= 600 or action_x <= -50 or action_y >= 100 or action_y <= -100:
            action_node = (-1000, -1000, -1000)
        action_theta = action_node[2]
        rounded_x = int(round(action_x))
        rounded_y = int(round(action_y))
        action_coord = (rounded_x, rounded_y)
        theta_index = int(action_theta / 30)

        # Determining new cost-to-go, cost-to-come, and total cost
        distance_to_goal = ((goal_x - action_x) ** 2 + (goal_y - action_y) ** 2) ** 0.5

        new_cost_to_come = parent_node_cost + action_dict[j][1]
        new_cost = new_cost_to_come + distance_to_goal

        if action_dict[j][0] in closed_list_check or action_coord in obstacle_list or action_node == (-1000, -1000, -1000) or \
                visited_nodes[rounded_y][rounded_x][theta_index] == 1:  # obs_check==True:
            match = True

        else:
            if action_node == (-1000, -1000, -1000):
                visited_nodes[rounded_y][rounded_x][theta_index] = 1
                visited_nodes[rounded_y+1][rounded_x][theta_index] = 1
                visited_nodes[rounded_y-1][rounded_x][theta_index] = 1
                visited_nodes[rounded_y][rounded_x+1][theta_index] = 1
                visited_nodes[rounded_y][rounded_x-1][theta_index] = 1
                visited_nodes[rounded_y+1][rounded_x+1][theta_index] = 1
                visited_nodes[rounded_y-1][rounded_x+1][theta_index] = 1
                visited_nodes[rounded_y+1][rounded_x-1][theta_index] = 1
                visited_nodes[rounded_y-1][rounded_x-1][theta_index] = 1
            # Adds node to node map and open list if nearest node has not been visited (zero value)
            if visited_nodes[rounded_y][rounded_x][theta_index] == 0:
                cost_to_come[action_dict[j][0]] = {'x': action_x, 'y': action_y, 'theta': action_theta,
                                                   'parent node': parent_node, 'cost to come': new_cost_to_come,
                                                   'total cost': new_cost, 'previous move': j}
                queue.append([cost_to_come[action_dict[j][0]]['total cost'], action_dict[j][0]])
                visited_nodes[rounded_y][rounded_x][theta_index] = 1
                visited_nodes[rounded_y + 1][rounded_x][theta_index] = 1
                visited_nodes[rounded_y - 1][rounded_x][theta_index] = 1
                visited_nodes[rounded_y][rounded_x + 1][theta_index] = 1
                visited_nodes[rounded_y][rounded_x - 1][theta_index] = 1
                visited_nodes[rounded_y + 1][rounded_x + 1][theta_index] = 1
                visited_nodes[rounded_y - 1][rounded_x + 1][theta_index] = 1
                visited_nodes[rounded_y + 1][rounded_x - 1][theta_index] = 1
                visited_nodes[rounded_y - 1][rounded_x - 1][theta_index] = 1
    counter = counter + 1
    # print(counter)
# This loop creates the reverse path by searching for the next parent node until the start node's parent "NA" is found
forward_path = generate_path(reverse_path)

end = time.time()
print(end - start)

cv2.imshow("Zeros matx", visual_map)  # show numpy array
cv2.waitKey(0)  # wait for ay key to exit window
cv2.destroyAllWindows()  # close all windows

x = []
y = []
visual_map_explore = visual_map
forward = False
forward_path.pop(-1)
counter = 0
for i in range(len(closed_list)):
    coord = closed_list[i]

    nodes_to_plot(coord)

    x_arr = (200 - int(((coord[1]+100))))
    y_arr = int((coord[0]+50) )
    visual_map_explore[x_arr][y_arr] = 0
    if counter == 10 or i == (len(closed_list) - 1):
        cv2.imshow("Zeros matx", visual_map_explore)  # show numpy array
        cv2.waitKey(1)  # wait for ay key to exit window
        counter = 0
    else:
        counter = counter + 1
# cv2.waitKey(0)  # wait for ay key to exit window
x = []
y = []
forward = True

for i in range(len(forward_path)):
    coord = forward_path[i]
    x_arr = (200 - int(((coord[1]+100))))
    y_arr = int((coord[0]+50))
    y = (200 - int(((coord[1]+100))))
    x = int((coord[0]+50) )

    if coord != start_node:
        forward_path_plot(coord)

    visual_map[x_arr, y_arr, :] = [0, 255, 0]
    cv2.imshow("Zeros matx", visual_map)  # show numpy array
    cv2.waitKey(1)  # wait for ay key to exit window
    prev_node = (x, y)

    pt1 = (2 * int(start_x), 500 - (int(start_y)) * 2)
    pt2 = (2 * int(goal_x), 500 - (int(goal_y)) * 2)

    cv2.arrowedLine(visual_map_explore, pt1, pt2, (0, 255, 0), 1)

cv2.waitKey(0)  # wait for ay key to exit window

cv2.destroyAllWindows()  # close all windows


