"""
Step 7 Smooth or Simplify geometry (I think they are same target)

Shapely provide built in method to simplify the geometry, however,
before using that method, it is important to merge road segment into
polylines, because now each network edge is represented by a line segment.

How to merge line segments into polylines?
1 - Remove all the degree 3 nodes and edges connecting those nodes
2 - Find connected components from the remaining graph
3 - Add the removed edges back to a cluster, if exists
    If the edge couldn't find a cluster to belong to, create a new cluster
4 - At this stage, all the road edges in a cluster should be merged into
    a gigantic polyline edge.
"""

import fiona
import sys
import os
import networkx as nx
import psycopg2
from shapely.wkt import loads
from shapely.geometry import LineString, Polygon, Point, MultiLineString
from shapely import ops
import numpy as np
import copy
import pickle
import readshp

eps = 0.0075
suffix = "0075"


def smooth(polyline):
    """
    This is a function to smooth the geometry of a polyline
    """
    coords = list(polyline.coords)
    if len(coords) == 2:  # which means it is a straight line
        return polyline  # we actually don't need to manipulate this
    else:
        new_coords = []
        new_coords.append(coords[0])
        for i in range(0, len(coords)-1):
            startPoint = coords[i]
            endPoint = coords[i+1]
            middleX = (startPoint[0] + endPoint[0]) / 2
            middleY = (startPoint[1] + endPoint[1]) / 2
            middlePoint = (middleX, middleY)
            new_coords.append(middlePoint)
        new_coords.append(coords[-1])
        return LineString(new_coords)

# Read the road network into memory
old_file_path = "result\\simu_roads\\T_simu_roads_" + suffix + ".shp"
old_roadnet = readshp.read_shp(old_file_path)

# Find all the nodes whose degree are 3
nodes_of_degree_3 = []
for node in old_roadnet.nodes():
    if old_roadnet.degree(node) == 3:
        nodes_of_degree_3.append(node)

# Find all the edges which will be auto removed when removing above nodes
removable_edges = []
for node in nodes_of_degree_3:
    # Find all the nodes connecting this degree 3 node
    neighbour_nodes = old_roadnet.neighbors(node)
    for this_node in neighbour_nodes:
        removable_edges.append((this_node, node))

# Remove all the nodes with degree 3
# Becareful, this will implicitly remove all the edges from these nodes
for node in nodes_of_degree_3:
    old_roadnet.remove_node(node)

# Get the each connected component now
graphs = list(nx.connected_component_subgraphs(old_roadnet))

"""
Now is an very important step
Add all the removable_edges back to the graph in graphs
"""
for edge in removable_edges:
    test_node = edge[0]
    find_graph = False
    graph_id = -1
    for i in range(len(graphs)):
        graph = graphs[i]
        if graph.has_node(test_node):
            find_graph = True
            graph_id = i
            break
    if find_graph:
        # if we find the graph this edge should belong to
        # add the edge from the graph
        graphs[i].add_edge(edge[0], edge[1])
    else:
        # we couldn't find a graph this edge should belong to
        new_graph = nx.Graph()
        new_graph.add_edge(edge[0], edge[1])
        graphs.append(new_graph)


poly_lines = []
for graph in graphs:
    if graph.number_of_edges() > 0:
        line_segments = []
        for edge in graph.edges():
            line_segments.append(LineString([edge[0], edge[1]]))
        multi_line = MultiLineString(line_segments)
        poly_line = ops.linemerge(multi_line)
        poly_lines.append(poly_line)

"""
Simplify geometry of roads
"""

new_poly_lines = []
for poly_line in poly_lines:
    new_poly_lines.append(smooth(poly_line))

"""
Decompose the poly_lines into line_segments
"""
line_segments = []
for new_poly_line in new_poly_lines:
    coords = list(new_poly_line.coords)
    for i in range(0, len(coords)-1):
        line_segments.append(LineString([coords[i], coords[i+1]]))

# Now let's see simplified poly_lines look like
sourceDriver = 'ESRI Shapefile'
sourceCrs = {'y_0': -100000, 'units': 'm', 'lat_0': 49,
             'lon_0': -2, 'proj': 'tmerc', 'k': 0.9996012717,
             'no_defs': True, 'x_0': 400000, 'datum': 'OSGB36'}

new_folder = "result/simu_roads"

sourceSchema = {'properties': {'Length': 'float:19.11'},
                'geometry': 'LineString'}
fileName = new_folder + '/T_simu_roads_' + suffix + '_simplify' + '.shp'
with fiona.open(fileName,
                'w',
                driver=sourceDriver,
                crs=sourceCrs,
                schema=sourceSchema) as source:
    for line_segment in line_segments:
        record = {}
        record['geometry'] = {'coordinates': list(line_segment.coords), 'type': 'LineString'}  # NOQA
        record['properties'] = {'Length': line_segment.length}  # NOQA
        source.write(record)
