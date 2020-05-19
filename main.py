import path_planning as path
import geopandas as pan
import shapely.geometry as ge
import shapely.ops as op
import matplotlib.pyplot as plt
import sys
import settings as st

st.world=pan.read_file('D:/UAV project/map.geojson')
st.world=st.world['geometry']
st.pstart=st.world[0]
st.pend=st.world[1]
st.obs=st.world[2:]
st.global_min=float('inf')
st.global_path=[]
st.global_sub_min=float('inf')
st.global_sub_path=[]

