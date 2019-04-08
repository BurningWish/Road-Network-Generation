"""
Step No.1

Using building Points to triangulate the whole area
"""

from scipy.spatial import Delaunay
import psycopg2
from shapely.geometry import Polygon
from shapely.wkt import loads
import matplotlib.pyplot as plt
import numpy as np


dbname = "aruptest1"
conn = psycopg2.connect("user = postgres password = 19891202 dbname = %s" % dbname)  # NOQA
cur = conn.cursor()
cur.execute("select st_astext(cp) from buildings order by bid")
results = cur.fetchall()
points = []

for result in results:
    wkt = result[0]
    points.append([loads(wkt).x, loads(wkt).y])

points = np.array(points)
tri = Delaunay(points)

plt.triplot(points[:,0], points[:,1], tri.simplices.copy())
plt.plot(points[:,0], points[:,1], 'o')

"""
Write the triangles to the DB
"""

cur.execute("drop table if exists triangles")
cur.execute("create table triangles (trid integer, geom geometry(Polygon, 27700), vxid integer[])")  # NOQA
tid = 0
for indices in tri.simplices:
    triangle = Polygon(points[indices])
    wkt = triangle.wkt
    cur.execute("insert into triangles (trid, geom, vxid) \
                values(%d, st_geomfromtext('%s', 27700), '{%d, %d, %d}')"
                % (tid, wkt, indices[0], indices[1], indices[2]))
    tid = tid + 1

conn.commit()
cur.close()
conn.close()
