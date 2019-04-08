"""
Step 4 Generating Inter Cluster Triangles using trimmed MST
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

f = open('serialize/trimMST_%s.txt' % str(eps), 'rb')
trimMST = pickle.load(f)
f.close()

f = open('serialize/removed_edges_%s.txt' % str(eps), 'rb')
rm_edges = pickle.load(f)
f.close()

graph_list = list(nx.connected_component_subgraphs(trimMST))

dbname = "aruptest1"
conn = psycopg2.connect("user = postgres password = 19891202 dbname = %s" % dbname)  # NOQA
cur = conn.cursor()

# Let's change this strategy, bidClusters should record building id
# for example [[1], [2,3], [4,5,6]]
bidClusters = []
for graph in graph_list:
    bidCluster = []
    for node in graph.nodes():
        wkt = Point(node).wkt
        cur.execute("select bid, st_distance(geom, st_geomfromtext('%s', 27700)) as dist \
                    from buildings \
                    where st_dwithin(geom, st_geomfromtext('%s', 27700), 5) \
                    order by dist" % (wkt, wkt))
        result = cur.fetchone()
        bid = result[0]
        bidCluster.append(bid)

    bidClusters.append(bidCluster)


# Need to read all the triangles again
cur.execute("select trid, vxid from filter_triangles \
            order by trid")
remain_trids = []
results = cur.fetchall()
for result in results:
    trid = result[0]
    bids = result[1]
    bid0, bid1, bid2 = bids
    for bidCluster in bidClusters:
        if bid0 in bidCluster:  # looks like one vertex is in a cluster
            if bid1 not in bidCluster or bid2 not in bidCluster:
                # at least one of the other vertex not in the same cluster  # NOQA
                remain_trids.append(trid)

tup_remain_trids = (tuple(remain_trids))


# write the remaining triangles back to the databases
cur.execute("drop table if exists remain_triangles_%s" % suffix)
cur.execute("create table remain_triangles_%s as select * from filter_triangles" % suffix)  # NOQA
cur.execute("delete from remain_triangles_%s \
            where trid not in %s" % (suffix, tup_remain_trids)
            )

conn.commit()

f = open("serialize/bidClusters_%s.txt" % str(eps), 'wb')
pickle.dump(bidClusters, f)
f.close()

cur.close()
conn.close()
