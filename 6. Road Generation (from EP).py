"""
Step 6

Draw the roads, finally...

However, here we start from entry point so that
we want to avoid the so called isolated loops
this is more time consuming and difficult
"""

import fiona
import sys
import os
import networkx as nx
import psycopg2
from shapely.wkt import loads
from shapely.geometry import LineString, Polygon, Point
import numpy as np
import copy
import pickle

eps = 0.0075
suffix = "0075"

f = open("serialize/bidClusters_%s.txt" % str(eps), 'rb')
bidClusters = pickle.load(f)
f.close()


dbname = "aruptest1"
conn = psycopg2.connect("user = postgres password = 19891202 dbname = %s" % dbname)  # NOQA
cur = conn.cursor()


def find_nearest_edge(pt, tri):
    tri_coords = list(tri.coords)
    edge0 = LineString([tri_coords[0], tri_coords[1]])
    edge1 = LineString([tri_coords[1], tri_coords[2]])
    edge2 = LineString([tri_coords[2], tri_coords[0]])
    min_dist = 10000000000000
    nearest_edge = edge0
    edges = [edge0, edge1, edge2]
    for edge in edges:
        current_dist = pt.distance(edge)
        if current_dist <= min_dist:
            min_dist = current_dist
            nearest_edge = edge
    
    return nearest_edge

def edgeLengthDiff(edge0, edge1):
    line0 = LineString(edge0)
    line1 = LineString(edge1)
    return abs(line0.length - line1.length) / min(line0.length, line1.length)


def buildingDist(edge):
    """
    the function accept one parameter the edge

    it will calculate the distance between footprints of two cores buildings
    """
    p0 = edge[0]
    p1 = edge[1]
    point0 = Point(p0)
    point1 = Point(p1)
    wkt0 = point0.wkt
    wkt1 = point1.wkt
    cur.execute("select bid from buildings \
                where st_contains(geom, st_geomfromtext('%s',27700))" % wkt0)
    bid0 = cur.fetchall()[0][0]

    cur.execute("select bid from buildings \
                where st_contains(geom, st_geomfromtext('%s',27700))" % wkt1)
    bid1 = cur.fetchall()[0][0]

    cur.execute("select st_distance(b1.geom, b2.geom) \
                from buildings as b1, buildings as b2 \
                where b1.bid=%d and b2.bid=%d" % (bid0, bid1))
    return cur.fetchall()[0][0]


def midPoint(edge):
    """
    the function accepts an edge ((0,1), (2,3)), and return the mid point
    for example (1.5, 2.0)
    """

    p0 = edge[0]
    p1 = edge[1]
    return ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2)


def edgeWithinCluster(edge):
    """
    the function accepts one parameter edge, and returns true
    if the edge is bridging buildings within a same cluster
    it returns false if the edge is bridging two clusters
    """
    node0 = edge[0]
    node1 = edge[1]
    pt0 = Point(node0)
    pt1 = Point(node1)

    cur.execute("select bid from buildings \
                where st_dwithin(cp, st_geomfromtext('%s', 27700), 1) \
                order by st_distance(cp, st_geomfromtext('%s', 27700))" % (pt0.wkt, pt0.wkt))  # NOQA
    bid0 = cur.fetchall()[0][0]

    cur.execute("select bid from buildings \
                where st_dwithin(cp, st_geomfromtext('%s', 27700), 1) \
                order by st_distance(cp, st_geomfromtext('%s', 27700))" % (pt1.wkt, pt1.wkt))  # NOQA
    bid1 = cur.fetchall()[0][0]
    for bidCluster in bidClusters:
        if bid0 in bidCluster:
            # if the first bid is found in a cluster
            if bid1 in bidCluster:
                # if the second bid is in the same cluster!
                return True
            else:
                # if the second bid is not in the same cluster!
                return False


def filterEdgesStrict(edges):
    """
    the filter condition is that we strict filter away the edge that is
    connecting buildings in the SAME cluster. This is the most strict rule!
    """
    filtered_edges = []
    for edge in edges:
        if not edgeWithinCluster(edge):
            filtered_edges.append(edge)

    return filtered_edges


def filterEdgesFinal(edges):
    """
    Make sure the triangle edge is not connecting buildings in the same cluster
    """

    # first make sure that edges are not inner cluster edges
    edges = filterEdgesStrict(edges)

    return edges


class Tri:
    def __init__(self, id):
        self.id = id
        self.edges = []
        self.filteredEdges = []


cur.execute("select trid, st_astext(st_exteriorring(geom)) \
            from remain_triangles_%s \
            order by trid" % suffix)  # NOQA

results = cur.fetchall()
triList = []

# first dump all the data into the triList
for result in results:
    tri = Tri(result[0])
    wkt = result[1]
    tempLine = loads(wkt)
    coords = list(tempLine.coords)
    edge0 = [coords[0], coords[1]]
    edge1 = [coords[1], coords[2]]
    edge2 = [coords[2], coords[0]]
    tri.edges = [edge0, edge1, edge2]
    triList.append(tri)

# calculate the filtered edges for each tri
for tri in triList:
    tri.filteredEdges = filterEdgesFinal(tri.edges)


# finally iterate throught the triList again and this time draw road edges
roadList = []
for tri in triList:
    if len(tri.filteredEdges) == 2:
        # easiest situation, and this is the most common one
        [edge0, edge1] = tri.filteredEdges
        mp0 = midPoint(edge0)
        mp1 = midPoint(edge1)
        linex = LineString([mp0, mp1])
        roadList.append(linex)

    elif len(tri.filteredEdges) == 3:
        
        # ok we discard traditional Y structure road completely
        # 90 degree cross road construction
        [edge0, edge1, edge2] = tri.filteredEdges
        # find the two edges that result the least length difference
        diff01 = edgeLengthDiff(edge0, edge1)
        diff12 = edgeLengthDiff(edge1, edge2)
        diff20 = edgeLengthDiff(edge2, edge0)
        diffs = [diff01, diff12, diff20]

        if diff01 == min(diffs):
            similarEdges = [edge0, edge1]
            specialEdge = edge2
        elif diff12 == min(diffs):
            similarEdges = [edge1, edge2]
            specialEdge = edge0
        else:
            similarEdges = [edge2, edge0]
            specialEdge = edge1

        # Now first calculate mp0 and mp1 using similarEdges
        mp0 = midPoint(similarEdges[0])
        mp1 = midPoint(similarEdges[1])
        # Calculate the mid point between mp0 and mp1
        mpx = midPoint((mp0, mp1))
        # Calculate the final point mp2
        mp2 = midPoint(specialEdge)
        roadList.append(LineString([mp0, mpx]))
        roadList.append(LineString([mp1, mpx]))
        roadList.append(LineString([mp2, mpx]))
        
        
        """
        [edge0, edge1, edge2] = tri.filteredEdges
        mp0 = midPoint(edge0)
        mp1 = midPoint(edge1)
        mp2 = midPoint(edge2)
        cp = ((mp0[0] + mp1[0] + mp2[0])/3, (mp0[1] + mp1[1] + mp2[1])/3)
        roadList.append(LineString([mp0, cp]))
        roadList.append(LineString([mp1, cp]))
        roadList.append(LineString([mp2, cp]))
        """


"""
Now use entry points to generate road segment
"""
cur.execute("select distinct on(epid) epid, tr.trid, st_astext(ep.geom), \
            st_astext(st_exteriorring(tr.geom)), st_distance(ep.geom, tr.geom) as distance \
            from entry_points as ep, remain_triangles_%s as tr \
            where st_dwithin(ep.geom, tr.geom, 1000) \
            order by epid, distance" % suffix)

results = cur.fetchall()
for data in results:
    point_wkt = data[2]
    tri_wkt = data[3]
    pt = loads(point_wkt) # shapely Point now
    tri = loads(tri_wkt) # shapely LineString now
    edge = find_nearest_edge(pt, tri)
    pt1 = (pt.x, pt.y)
    pt2 = midPoint(list(edge.coords))
    roadList.append(LineString([pt1, pt2]))


# Now let's see how generated road looks like?
sourceDriver = 'ESRI Shapefile'
sourceCrs = {'y_0': -100000, 'units': 'm', 'lat_0': 49,
             'lon_0': -2, 'proj': 'tmerc', 'k': 0.9996012717,
             'no_defs': True, 'x_0': 400000, 'datum': 'OSGB36'}

new_folder = "result/simu_roads"

sourceSchema = {'properties': {'Length': 'float:19.11'},
                'geometry': 'LineString'}
fileName = new_folder + '/T_simu_roads_%s' % suffix + '.shp'
with fiona.open(fileName,
                'w',
                driver=sourceDriver,
                crs=sourceCrs,
                schema=sourceSchema) as source:
    for road in roadList:
        coords = list(road.coords)
        record = {}
        record['geometry'] = {'coordinates': coords, 'type': 'LineString'}  # NOQA
        record['properties'] = {'Length': road.length}  # NOQA
        source.write(record)

cur.close()
conn.close()
