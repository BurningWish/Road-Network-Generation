"""
We used Global STD based ways to create clusters

However, my friend, however, in order to decide the best edge,

I add one more check, that is the best edge cannot be too short.
"""

import fiona
import sys
import os
import networkx as nx
import psycopg2
from shapely.wkt import loads
from shapely.geometry import LineString, Point
import numpy as np
import copy
import pickle

"""
========================    Functions to be Called    =========================
"""

eps = 0.0075


def calculate_std(graph):
    """
    This method only accepts one parameter, a graph instance.

    We calculate the standard deviation on the edge length of that graph,
    and we will return that value

    the method can even deal with grpah who has 0 or 1 edge.

    the idea is that this method return false if this graph has 0 or 1 edge.
    Becasue calculating std for a sample size 0 or 1 is meaningless.
    """

    if graph.number_of_edges() == 0 or graph.number_of_edges() == 1:
        return False

    edge_lengths = []
    for edge in graph.edges():
        startNode = edge[0]
        endNode = edge[1]
        edge_lengths.append(graph[startNode][endNode]['Boundary Distance'])

    return np.std(edge_lengths)


def calculate_std_graph_list(graph_list):
    """
    This method accepts one parameter, a list containing graphs.
    
    for each graph in the list, calculate its own std,
    then each std is weighted by the number of edges of that graph.
    
    Finally we got overall std on this graph_list
    
    By the way, only accept a graph that has at least 2 edges
    
    the reason is that if it has 0 edge, the std is NAN
    if it has 1 edge, the std is 0
    """
    sum_weighted_std = 0
    sum_edge_number = 0
    for graph in graph_list:
        
        this_edge_number = graph.number_of_edges()
        if this_edge_number >= 2:
            sum_edge_number += this_edge_number
            this_std = calculate_std(graph)
            sum_weighted_std += this_std * graph.number_of_edges()
    return sum_weighted_std / sum_edge_number

    
def find_best_edge(tree):
    """
    Instead of finding the longest edge, we want to find a edge
    so that removing this edge will contribute to the largest
    overall std reduction
    """
    
    """
    My additional input is quite funny my friend
    """
    
    graph_list = list(nx.connected_component_subgraphs(tree))
    tree_std = calculate_std_graph_list(graph_list)
    
    max_std_reduction = 0
    best_edge = [(0,0), (1,1)]
    for edge in tree.edges():
        copy_tree = copy.deepcopy(tree)
        startNode = edge[0]
        endNode = edge[1]
        
        # Yeah my friend, this is something new and interesting
        edgeLength = tree[startNode][endNode]['Boundary Distance']
        
        copy_tree.remove_edge(startNode, endNode)
        copy_graph_list = list(nx.connected_component_subgraphs(copy_tree))
        
        current_std = calculate_std_graph_list(copy_graph_list)
        current_std_reduction = tree_std - current_std
        
        no_small_clusters = True
        
        # make sure if we remove this edge, no small cluster will be created
        for graph in copy_graph_list:
            if graph.number_of_nodes() <= 3:
                no_small_clusters = False
                break
        
        # Yeah, just add one more check my friend, things are completely different
        if current_std_reduction > max_std_reduction and edgeLength > 5 and no_small_clusters:  # NOQA
            max_std_reduction = current_std_reduction
            best_edge = [startNode, endNode]
        copy_tree = None
        copy_graph_list = None
            
#    print("max_std_drop will be: ", max_std_reduction)
    startNode = best_edge[0]
    endNode = best_edge[1]
#    print("this edge length is: ", tree[startNode][endNode]['Length'])
    return best_edge

    


"""
========================    Actual Code    ====================================
"""

dbname = "aruptest1"
conn = psycopg2.connect("user = postgres password = 19891202 dbname = %s" % dbname)  # NOQA
cur = conn.cursor()
cur.execute("select st_astext(st_exteriorring(geom)) from triangles \
            order by trid")
results = cur.fetchall()

triNetwork = nx.Graph()
for result in results:
    wkt = result[0]
    linestring = loads(wkt)
    coords = list(linestring.coords)
    for i in range(3):  # i will be 0, 1, 2
        startVertex = coords[i]
        endVertex = coords[i+1]
        
        startNode = startVertex
        endNode = endVertex
        
        
        if not triNetwork.has_edge(startNode, endNode):
            # which means this edge doesn't exist and we will add it
            attributes = {}
            attributes['Coords'] = [startVertex, endVertex]
            attributes['Centroid Distance'] = LineString([startVertex, endVertex]).length
            triNetwork.add_edge(startNode, endNode, attributes)

"""
At this stage triNetwork has attribute centroid distance, enough for old MST

but we want to calculate the boundary distance and save it to each edge
"""

for edge in triNetwork.edges():
    startNode = edge[0]
    endNode = edge[1]
    startPoint = Point(startNode)
    endPoint = Point(endNode)
    startwkt = startPoint.wkt
    endwkt = endPoint.wkt
    cur.execute("select bid from buildings \
                where st_intersects(geom, st_geomfromtext('%s', 27700))" % startwkt)
    bid1 = cur.fetchall()[0][0]
    cur.execute("select bid from buildings \
                where st_intersects(geom, st_geomfromtext('%s', 27700))" % endwkt)
    bid2 = cur.fetchall()[0][0]
    cur.execute("select st_distance(b1.geom, b2.geom) \
                from buildings as b1, buildings as b2 \
                where b1.bid = %d and b2.bid = %d" % (bid1, bid2))
    boundary_distance = cur.fetchall()[0][0]
    triNetwork[startNode][endNode]['Boundary Distance'] = boundary_distance
            
"""
======
Generate the minimum spanning tree
======
"""

tree = nx.minimum_spanning_tree(triNetwork, weight="Boundary Distance")

"""
======
Visualize the MST
======
"""

"""
sourceDriver = 'ESRI Shapefile'
sourceCrs = {'y_0': -100000, 'units': 'm', 'lat_0': 49,
             'lon_0': -2, 'proj': 'tmerc', 'k': 0.9996012717,
             'no_defs': True, 'x_0': 400000, 'datum': 'OSGB36'}

new_folder = "shp/"

sourceSchema = {'properties': 
                    {'CenDist': 'float:19.11', 
                    'BoundDist': 'float: 19.11'},
                'geometry': 'LineString'}
fileName = new_folder + '/New_Minimum_Spanning_Tree' + '.shp'
with fiona.open(fileName,
                'w',
                driver=sourceDriver,
                crs=sourceCrs,
                schema=sourceSchema) as source:

        for edge in tree.edges():
            startNode = edge[0]
            endNode = edge[1]
            record = {}
            record['geometry'] = {'coordinates': tree.edge[startNode][endNode]['Coords'], 'type': 'LineString'}  # NOQA
            record['properties'] = {'CenDist': tree.edge[startNode][endNode]['Centroid Distance'],
                                    'BoundDist': tree.edge[startNode][endNode]['Boundary Distance']}  # NOQA
            source.write(record)
"""


"""
+++
ok, it's time to test if our new cluster algorithm using MST
+++
"""
last_std = calculate_std(tree)
initial_std = last_std
print("inital std on MST is: ", last_std)
print("\n")

# we need this information about the edges that have been removed from the tree
removed_edges = []

break_loop = False
i = 0
while not break_loop:
    print("Loop No.", i)
    best_edge = find_best_edge(tree)
    removed_edges.append(best_edge)
    startNode = best_edge[0]
    endNode = best_edge[1]
    print("we remove edge with length: ", tree[startNode][endNode]['Boundary Distance'])
    tree.remove_edge(startNode, endNode)
    graph_list = list(nx.connected_component_subgraphs(tree))
    next_std = calculate_std_graph_list(graph_list)
    print("after removing we have %d subgraphs" % len(graph_list))
    print("Now the overall std for the current tree is: ", next_std)
    print("std Reduction is: ", last_std - next_std)
    total_std_drop = initial_std - next_std
    iteration_checker = eps * (total_std_drop + 1)
    print("iteration checker is: ", iteration_checker)
    if last_std - next_std < iteration_checker:
        print("Break the loop")
        break_loop = True
    last_std = next_std
    i += 1
    print('\n')


"""
Let's draw the truly dismembered MST to see if it makes any sense:)
"""


sourceDriver = 'ESRI Shapefile'
sourceCrs = {'y_0': -100000, 'units': 'm', 'lat_0': 49,
             'lon_0': -2, 'proj': 'tmerc', 'k': 0.9996012717,
             'no_defs': True, 'x_0': 400000, 'datum': 'OSGB36'}

new_folder = "result/cluster"

sourceSchema = {'properties': 
                        {'CenDist': 'float:19.11', 
                         'BoundDist': 'float: 19.11'},
                'geometry': 'LineString'}
fileName = new_folder + '/trim_MST_modified_%s' % str(eps) + '.shp'
with fiona.open(fileName,
                'w',
                driver=sourceDriver,
                crs=sourceCrs,
                schema=sourceSchema) as source:

    for edge in tree.edges():
        startNode = edge[0]
        endNode = edge[1]
        record = {}
        record['geometry'] = {'coordinates': tree.edge[startNode][endNode]['Coords'], 'type': 'LineString'}  # NOQA
        record['properties'] = {'CenDist': tree.edge[startNode][endNode]['Centroid Distance'],
                                    'BoundDist': tree.edge[startNode][endNode]['Boundary Distance']}  # NOQA
        source.write(record)

       
"""
Serialize the trimmed MST
"""
f = open("serialize/trimMST_%s.txt" % str(eps), 'wb')
pickle.dump(tree, f)
f.close()

f = open("serialize/removed_edges_%s.txt" % str(eps), 'wb')
pickle.dump(removed_edges, f)
f.close()


cur.close()
conn.close()