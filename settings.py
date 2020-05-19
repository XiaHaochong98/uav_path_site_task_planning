import geopandas as pan
world=pan.read_file('D:/UAV project/map.geojson')
world=world['geometry']
pstart=world[1]
pend=world[0]
obs=world[2:]
global_min=float('inf')
global_path=[]
global_sub_min=float('inf')
global_sub_path=[]

def reset():
    global world
    global pstart
    global pend
    global obs
    global global_min
    global global_path
    global global_sub_min
    global global_sub_path

    # world = None
    # pstart = None
    # pend = None
    # obs = None
    global_min = float('inf')
    global_path = []
    global_sub_min = float('inf')
    global_sub_path = []
