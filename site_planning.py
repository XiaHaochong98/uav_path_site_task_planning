#0.05=5km
#fmax=10km fsafe=7km fcover=3.5km
import geopandas as pan
import shapely.geometry as ge
import shapely.ops as op
import matplotlib.pyplot as plt
import sys
sys.setrecursionlimit(1500)
def p(geos,color):
    base=geos[0].plot()
    for i, geo in enumerate(geos[1:]):
        base=geo.plot(ax=base,color=color[i])
    plt.show()
world=pan.read_file('D:/UAV project/map.geojson')
world=world['geometry']
boundary=world.boundary
pstart=world[0]
pend=world[1]
obs=world[2:]
covered=[0 for _ in range(len(obs))]
