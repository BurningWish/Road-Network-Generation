"""
Step No 3.

this step doesn't care about epsilon......

okay, I am pretty sure we need to do 2 filterings.

(1) filter away the triangles whose smallest angle is too small (< 10 deg)

(2) filter away the triangles that not fall within the concave hull
"""

import fiona
import sys
import os
import networkx as nx
import psycopg2
from shapely.wkt import loads
from shapely.geometry import LineString, Point, Polygon, MultiPolygon
import numpy as np
import copy
from math import acos, degrees

def calculate_smallest_angle(a, b, c):
    """
    This function accepts three parameters,
    which are the lengths of 3 edges of a triangle
    
    it will return the smalles angle in the angle
    in the unit of degree
    """
    angle_c = degrees(acos((a * a + b * b - c * c)/(2.0 * a * b)))
    angle_b = degrees(acos((a * a + c * c - b * b)/(2.0 * a * c)))
    angle_a = degrees(acos((b * b + c * c - a * a)/(2.0 * b * c)))
    
    return min(angle_a, angle_b, angle_c)

dbname = "aruptest1"
conn = psycopg2.connect("dbname = %s user = postgres password = 19891202" % dbname)
cur = conn.cursor()

cur.execute("drop table if exists filter_triangles")

"""
Step 1 - Find all the triangles that have not so small angle
"""
conn.commit()

step1_trids = []

cur.execute("select trid, st_astext(st_exteriorring(geom)) from triangles \
            order by trid")
results = cur.fetchall()

for result in results:
    trid = result[0]
    wkt = result[1]
    linestring = loads(wkt)
    coords = list(linestring.coords)
    polygon = Polygon(coords)
    new_wkt = polygon.wkt
    a = LineString([coords[0], coords[1]]).length
    b = LineString([coords[1], coords[2]]).length
    c = LineString([coords[2], coords[0]]).length
    min_angle = calculate_smallest_angle(a, b, c)
    if min_angle > 10:
        step1_trids.append(trid)
        

"""
Step 2 - Find all triangles that fall within the concave hull
"""
step2_trids = []

cur.execute("select trid from triangles as tr, concave_hull as ch \
            where chid = 0 and st_area(st_intersection(tr.geom, ch.geom)) / st_area(tr.geom) > 0.99 \
            order by trid")
results = cur.fetchall()
for result in results:
    step2_trids.append(result[0])

    
"""
Step 3 - Find the the triangles you have filtered and insert them into PostGIS
"""
filter_trids = [trid for trid in step1_trids if trid in step2_trids]
cur.execute("create table filter_triangles as select * from triangles")
conn.commit()
for trid in filter_trids:
    cur.execute("delete from filter_triangles where trid not in %s" % ((tuple(filter_trids)), ))
conn.commit()

cur.close()
conn.close()