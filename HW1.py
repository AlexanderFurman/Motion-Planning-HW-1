import argparse
import os
from typing import List, Tuple
import numpy as np

from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString, orient
import copy
import math

#added for compare
#from quantimpy import minkowski as mk

def angle_vector():
    return

# TODO
def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """

    #calculate Q object:
    Q=Polygon([(0 ,-r),(r, 0),(0 ,r),(-r, 0)]) #(-r, 0)]
    m=4
    n=original_shape.boundary.bounds.__len__()
    i=0
    j=0
    v=original_shape
    v=orient(v, sign=1.0)
    #shapely.geometry.polygon.orient(polygon, sign=1.0)

    PQ=[]
    cond=0
    while cond==0:
        if(i>=n and j>=m):
            break
        PQ.append((v.boundary.coords.xy[0][i%n]+Q.boundary.coords.xy[0][j%m],v.boundary.coords.xy[1][i%n]+Q.boundary.coords.xy[1][j%m]))
        v_dx=(v.boundary.coords.xy[0][(i+1)%n]-v.boundary.coords.xy[0][i%n])
        v_dy=v.boundary.coords.xy[1][(i+1)%n]-v.boundary.coords.xy[1][i%n]
        #angle_v=(v_dy)/(v_dx)
        w_dx=(Q.boundary.coords.xy[0][(j+1)%m]-Q.boundary.coords.xy[0][j%m])
        w_dy=Q.boundary.coords.xy[1][(j+1)%m]-Q.boundary.coords.xy[1][j%m]
        #angle_w = (w_dy)/(w_dx)
        cross=v_dx*w_dy-v_dy*w_dx
        if cross>=0:#angle_v<angle_w:
            i+=1
        if cross<=0:#angle_v>angle_w:
            j+=1

    print('minkowsky')
    v_new=Polygon(PQ)
    print(v_new.bounds)
    return v_new

    #pass


# TODO
def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """

    #first preprocess all of the points:

    lines = []
    point_list = []
    # neighbours = {}
    # for point in point_list:
    #     neighbours[hash(point)] = [0]
    # print(neighbours.get_keys())

    #add all point tuples from the polygons into an unordered list
    for polygon in obstacles:
        num_vertices = len(polygon.exterior.coords.xy[1][1:])

        for i in range(num_vertices):
            point_list.append((polygon.exterior.coords.xy[0][i], polygon.exterior.coords.xy[1][i]))

    if source is not None:
        point_list.append(source)
    if dest is not None:
        point_list.append(dest)

    # For each point in point_list, create a LineString using every other point in point_list
    # and check if the LineString intersects with any of the polygons
    # If there is no intersection, we add the LineString to the lines array
    #NOTE: This is a naive approach, and takes O(n^3) (?) - Try find something better!!!

    for i in range(len(point_list)):
        for j in range(len(point_list)):
            if i == j:
                break
            edge = LineString([point_list[i], point_list[j]])
            flag = False
            for polygon in obstacles:
                if edge.crosses(polygon) or edge.covered_by(polygon):
                    flag = True
                    break
            if not flag:
                lines.append(edge)

    return lines


# def sort_points_by_angle(point, point_list):
#     # this would be used for a faster vis_graph algorithm
#     current_list = point_list.copy()
#     current_list.remove(point)
#     current_list.sort(key=angle_key)
#     return current_list


# def angle_key(x):
#     # this would be used for a faster vis_graph algorithm
#     atan = math.atan2(x[1], x[0])
#     return (atan, x[1]**2+x[0]**2) if atan >= 0 else (2*math.pi + atan, x[0]**2+x[1]**2)

#TODO shortest path algorithm
def dijkstra(lines):
    # First process edges into
    pass


def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)


def get_points_and_dist(line):
    source, dist = line.split(' ')
    dist = float(dist)
    source = tuple(map(float, source.split(',')))
    return source, dist


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("Robot", help="A file that holds the starting position of the robot, and the distance from the center of the robot to any of its vertices")
    parser.add_argument("Obstacles", help="A file that contains the obstacles in the map")
    parser.add_argument("Query", help="A file that contains the ending position for the robot.")
    args = parser.parse_args()
    obstacles = args.Obstacles
    robot = args.Robot
    query = args.Query
    is_valid_file(parser, obstacles)
    is_valid_file(parser, robot)
    is_valid_file(parser, query)
    workspace_obstacles = []
    with open(obstacles, 'r') as f:
        for line in f.readlines():
            ob_vertices = line.split(' ')
            if ',' not in ob_vertices:
                ob_vertices = ob_vertices[:-1]
            points = [tuple(map(float, t.split(','))) for t in ob_vertices]
            workspace_obstacles.append(Polygon(points))
    with open(robot, 'r') as f:
        source, dist = get_points_and_dist(f.readline())

    #Alex Test:
    with open(query, 'r') as f:
        dest = tuple(map(float, f.readline().split(',')))
    lines = get_visibility_graph(workspace_obstacles, source, dest)
    plotter = Plotter()
    plotter.add_obstacles(workspace_obstacles)
    plotter.add_robot(source, dist)
    plotter.add_visibility_graph(lines)
    plotter.show_graph()


    # step 1:
    c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    plotter1 = Plotter()

    plotter1.add_obstacles(workspace_obstacles)
    plotter1.add_c_space_obstacles(c_space_obstacles)
    plotter1.add_visibility_graph(lines)
    print(f"workspace obstacles = {workspace_obstacles}")
    print(f"cspace obstacles = {c_space_obstacles}")

    print(f"workspace obstacle points = {workspace_obstacles[0].exterior.coords.xy}")
    print(f"cspace obstacle points = {c_space_obstacles[0].exterior.coords.xy}")
    plotter1.add_robot(source, dist)
    plotter1.show_graph()

    # step 2:

    lines = get_visibility_graph(c_space_obstacles)
    plotter2 = Plotter()
    plotter2.add_obstacles(workspace_obstacles)
    plotter2.add_c_space_obstacles(c_space_obstacles)
    plotter2.add_visibility_graph(lines)
    plotter2.add_robot(source, dist)

    plotter2.show_graph()

    # step 3:
    with open(query, 'r') as f:
        dest = tuple(map(float, f.readline().split(',')))

    lines = get_visibility_graph(c_space_obstacles, source, dest)
    #TODO: fill in the next line
    shortest_path, cost = None, None

    plotter3 = Plotter()
    plotter3.add_robot(source, dist)
    plotter3.add_obstacles(workspace_obstacles)
    plotter3.add_robot(dest, dist)
    plotter3.add_visibility_graph(lines)
    plotter3.add_shorterst_path(list(shortest_path))


    plotter3.show_graph()