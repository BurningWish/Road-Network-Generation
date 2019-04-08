"""
This is also preparation step

In fact, this step must be run before any other step.
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

dbname = "aruptest1"

conn = psycopg2.connect("dbname = %s user = postgres password = 19891202" % dbname)
cur = conn.cursor()

cur.execute("alter table buildings drop column if exists bid")
cur.execute("alter table buildings add column bid integer")
cur.execute("update buildings set bid = gid - 1")

cur.execute("alter table buildings drop column if exists cp")
cur.execute("alter table buildings add column cp geometry(Point, 27700)")
cur.execute("update buildings set cp = st_centroid(geom)")

cur.execute("alter table entry_points drop column if exists epid")
cur.execute("alter table entry_points add column epid integer")
cur.execute("update entry_points set epid = gid - 1")

conn.commit()

cur.close()
conn.close()