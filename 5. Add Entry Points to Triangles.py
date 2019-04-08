"""
Step 5

Alright let's add the entry points to the triangles.
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

eps = 0.0075
suffix = "0075"

def find_nearest_edge(pt, tri):
    """
    we accept two parameters, a shapely point and a shapely linestring (triangle),
    we want to find nearest edge
    """
    
    tri_coords = list(tri.coords)
    edge0 = LineString([tri_coords[0], tri_coords[1]])
    edge1 = LineString([tri_coords[1], tri_coords[2]])
    edge2 = LineString([tri_coords[2], tri_coords[3]])
    
    minDist = 100000000000
    nearestEdge = edge0
    edges = [edge0, edge1, edge2]
    
    for edge in edges:
        currentDist = pt.distance(edge)
        if currentDist < minDist:
            minDist = currentDist
            nearestEdge = edge
    
    return nearestEdge



dbname = "aruptest1"
conn = psycopg2.connect("user = postgres password = 19891202 dbname = %s" % dbname)  # NOQA
cur = conn.cursor()

cur.execute("drop table if exists base_triangles_%s" % suffix)
cur.execute("create table base_triangles_%s as \
            select * from remain_triangles_%s" % (suffix, suffix))
conn.commit()



cur.execute("select max(trid) from base_triangles_%s" % suffix)
max_trid = cur.fetchone()[0]
new_trid = max_trid

cur.execute("select distinct on(epid) epid, tr.trid, st_astext(ep.geom), \
            st_astext(st_exteriorring(tr.geom)), st_distance(ep.geom, tr.geom) as distance \
            from entry_points as ep, remain_triangles_%s as tr \
            where st_dwithin(ep.geom, tr.geom, 1000) \
            order by epid, distance" % suffix)

results = cur.fetchall()
for data in results:
    point_wkt = data[2]
    tri_wkt = data[3]
    pt = loads(point_wkt)  # shapely Point now
    tri = loads(tri_wkt)  # shapely LineString now
    edge = find_nearest_edge(pt, tri)

    pt1, pt2 = list(edge.coords)
    pt3 = (pt.x, pt.y)
    new_polygon = Polygon([pt1, pt2, pt3])
    new_wkt = new_polygon.wkt
    cur.execute("insert into base_triangles_%s (trid, geom) \
                values(%d, st_geomfromtext('%s', 27700))" % (suffix, new_trid, new_wkt))
    new_trid += 1
    conn.commit()

conn.commit()    
cur.close()
conn.close()