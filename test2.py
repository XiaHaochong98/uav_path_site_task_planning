import geopandas as pan
import matplotlib.pyplot as plt
import shapely.geometry as ge
# import path_planning as pa
import settings as st
def p(obj):
    pan.GeoSeries(obj).plot()
    plt.show()
g1=ge.Polygon([(0,0),(0,1),(1,1),(1,0)])
g2=ge.Polygon([(0,0),(1,0),(2,2),(0,1)])
l1=ge.LineString([(-1,-1),(0.5,0.5)])
l2=ge.LineString([(0,0),(1,1)])
l3=ge.LineString([(0,0),(-1,-1)])
i1=g1.exterior.intersection(l1)
# start=(0,0)
# end=(2,2)
# e1=g1.exterior.coords[:]
# e2=g2.exterior.coords[:]
# i1=g1.exterior.intersection(g2.exterior)
# stack=[]
# ind=e2.index(end)
# for i in range(ind-1,-1,-1):
#     if e2[i] in e1:
#         stack.append(e2[i])
#         break
# for i in range(ind+1,ind+len(e2)):
#     if i>=len(e2):
#         i-=len(e2)
#     if e2[i] in e1:
#         stack.append(e2[i])
#         break
# print(stack)
# new_start=[]
# for geo in i1:
#     stack.append(geo.coords[:])
# stack=i1.coords[:]
