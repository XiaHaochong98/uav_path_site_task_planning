# import geopandas as pan
# import matplotlib as plt
# world=pan.read_file('D:/UAV project/wholeworld.geojson')

import geopandas as pan
import matplotlib.pyplot as plt
import shapely.geometry as ge
import settings as st
import math
import csv
import path_planning_backup as pa
import statistics
from SetCoverPy import *
import numpy as np
import copy
import random
import itertools



# tip:first generate all coverage in every obstacle (2)use a matrix to memorize each obstcle stack cover
#(3)get a min-genrate set that cover all obstcle (4) add site to connect all obstacle site
#(4)calculate everage site number to start (5) find min everage in all possible site
#先用精确搜索确定最小p（coversite数量） 然后再随机生成p*k个，然后drop，然后回火启发确定最优解
flight_range=0.1
cover_range=flight_range/3#cover range
travel_range=flight_range*(2/3)#max distance betweens site to be connected
rdif=0.001
# def __init__():
def obs_cover():
    mat = [[0 for _ in range(len(st.obs))] for _ in range(len(st.obs))]
    for i,ob in enumerate(st.obs):
        print(i)
        points=point_coverage(ob.centroid)
        P=get_P(points)
        for j,ob_ in enumerate(st.obs):
            if P.intersects(ob_):
                mat[j][i]=1
    return mat
def csv_write(mat,filename):
    with open("D:/UAV project/"+filename+".csv", "w+") as my_csv:
        csvWriter = csv.writer(my_csv, delimiter=',')
        csvWriter.writerows(mat)
    print(filename,'written')
    return
def csv_read(name):
    with open('./'+name+'.csv', newline='\n') as f:
        reader = csv.reader(f)
        mat = list(reader)
    return mat
def get_circle(point,r):
    circle=ge.MultiPoint(ge.LineString(point.buffer(r).exterior).coords[:])
    return circle
def point_coverage(point,mode='cover'):
    global cover_range
    global travel_range
    global obs
    # global rdif
    points=[None for _ in range(65)]
    if mode=='cover':
        nowr=cover_range
    else:
        nowr=travel_range
    rdif=(nowr*3)/100
    while not all(points):
        # print(nowr)
        if nowr<=0:
            break
        circle=get_circle(point,nowr)
        # print(circle)
        for i , circle_point in enumerate(circle):
            if not points[i]:
                # print('input',point,circle_point)
                # print(nowr)
                st.reset()
                # print(point,'\n',circle_point)
                dis,path=pa.find(point,circle_point,0,[])
                # print(dis,st.global_min)
                # if i==65:
                    # stack.append([point,circle_point])
                if min(dis,st.global_min)<=cover_range:
                    points[i]=circle_point
        nowr-=rdif
    return points
def cal_all_coverage():
    stack=[]
    i=0
    for ob in st.obs:
        i+=1
        print(i)
        temp=[]
        for point in point_coverage(ob.centroid):
            temp.append(point.x)
            temp.append(point.y)
        stack.append(temp)
        print('len',len(stack[-1]))
    with open("D:/UAV project/obs_point.csv", "w+") as my_csv:
        csvWriter = csv.writer(my_csv, delimiter=',')
        csvWriter.writerows(stack)
        print('written')
    return
def ob_dis():
    stack=[0 for _ in range(len(st.obs))]
    for i,ob in enumerate(st.obs):
        # print(i)
        st.reset()
        lenght, path = pa.find(st.pstart,ob.centroid, 0, [])
        stack[i]=min(lenght,st.global_min)
    with open("D:/UAV project/obs_distance.csv", "w+") as my_csv:
        csvWriter = csv.writer(my_csv, delimiter=',')
        csvWriter.writerow(stack)
    print('obs_distance to start generated')
    return
def get_obs_coord():
    stack=[]
    for ob in st.obs:
        po=ob.centroid
        stack.append(po[0])
        stack.append(po[1])
    with open("D:/UAV project/obs_point.csv", "w+") as my_csv:
        csvWriter = csv.writer(my_csv, delimiter=',')
        csvWriter.writerow(stack)
    print('sites coord generated')
    return
def get_P(points):
    return ge.Polygon([(point.x,point.y) for point in points])
def get_R(points):
    return ge.LineString([(point.x, point.y) for point in points]+[(points[0].x,points[1].y)])
def sp(ge):
    ge.plot()
    plt.show()
def p(geos,color):
    base=geos[0].plot()
    for i, geo in enumerate(geos[1:]):
        base=geo.plot(ax=base,color=color[i])
    plt.show()
def find_plot(objects):
    p([st.world,pan.GeoSeries(objects)],['red'])
def normalize_cost(cost,offset):
    res=[c+offset for c in cost]
    me=statistics.median(res)
    res=[c/me for c in res]
    return res
def add_start_cover(mat):
    points=point_coverage(st.pstart)
    P=get_P(points)
    for j,ob_ in enumerate(st.obs):
        if P.intersects(ob_):
            for i in range(len(mat[0])):
                mat[j][i]=1
    return mat
def p(geos,color):
    base=geos[0].plot()
    for i, geo in enumerate(geos[1:]):
        base=geo.plot(ax=base,color=color[i])
    plt.show()
def pick_sites(greed_flag=0,maxiters_set=20,show=0):
    cover_mat = csv_read('obs_cover_real_addstart')
    for i in range(len(cover_mat)):
        for j in range(len(cover_mat[0])):
            cover_mat[i][j] = int(cover_mat[i][j])
    dis_mat = csv_read('obs_distance')[0]
    obs_point = csv_read('obs_point')
    for i in range(len(dis_mat)):
        dis_mat[i] = float(dis_mat[i])
    solver = setcover.SetCover(np.array(cover_mat), np.array(dis_mat), maxiters=maxiters_set)
    if greed_flag:
        res = solver.greedy()
        select = solver.s
    else:
        res, time, u, select = solver.SolveSCP()
    seleted_points_ring = []
    seleted_points = [st.pstart]
    for i, s in enumerate(select):
        if s:
            seleted_points.append(ge.Point(st.obs[i + 2].centroid))
            temp = []
            # print(len(obs_point[i]))
            for j in range(0, len(obs_point[i]) // 2):
                # print(j*2)
                temp.append(ge.Point(float(obs_point[i][j * 2]), float(obs_point[i][j * 2 + 1])))
            seleted_points_ring.append(get_R(temp))
    seleted_points_ring.append(get_R(point_coverage(st.pstart)))
    print('sites picked')
    if show==1:
        p([st.obs, pan.GeoSeries(seleted_points_ring), pan.GeoSeries(seleted_points)], ['red', 'black'])
    return seleted_points
def picked_sites_dis(selected_points_this):
    # global ref
    # ref=selected_points_this
    dis_mat=[[-1 for _ in range(len(selected_points_this))] for _ in range(len(selected_points_this))]
    i=0
    while i <len(selected_points_this)-1:
        # print('i',i,len(selected_points_this))
        for j in range(i+1,len(selected_points_this)):
            st.reset()
            # print(i,j)
            # print(type(i),type(j))
            # print(selected_points_this[i],'\n',selected_points_this[j])
            # print(type(selected_points_this[i]),type(selected_points_this[j]))
            lenght, path = pa.find(selected_points_this[i], selected_points_this[j], 0, [])
            # print('min',st.global_min,'lenght',lenght,'\n')
            dis_mat[i][j]=min(st.global_min,lenght)
        # print('pre',i,len(selected_points_this))
        i+=1
    # csv_write(dis_mat,"picked_sites_dis")
    print('sites dis calculated')
    return dis_mat
def picked_sites_MST(picked_sites_dis, punish=0.2):
    #add edge if dis<travel_range/give punish to the tree length
    global travel_range
    # picked_sites_dis=csv_read("picked_sites_dis")
    def prim(mat,punish):
        # real_mat=copy.deepcopy(mat)
        union_tree=[i for i in range(len(mat))]
        tier=[-1 for _ in range(len(mat))]
        visited=[0 for _ in range(len(mat))]
        edge=[]
        open=[]
        visited[0]=1
        tier[0]=0
        for i in range(1,len(mat)):
            if mat[0][i]<=travel_range:
                edge.append([0,i])
                open.append(i)
                visited[i]=mat[0][i]
                tier[i]=1
                union_tree[i]=0
        # print(visited)
        while not all(visited):
            # print(visited)
            mi=float('inf')
            mi_edge=[]
            for i in open:
                for j in range(0,i):
                    if not visited[j] and mat[j][i]<mi:
                        mi=mat[j][i]
                        mi_edge=[i,j]
                for j in range(i+1,len(mat)):
                    if not visited[j] and mat[i][j]<mi:
                        mi=mat[i][j]
                        mi_edge=[i,j]
            visited[mi_edge[1]]=visited[mi_edge[0]]+mi
            tier[mi_edge[1]]=tier[mi_edge[0]]+1
            union_tree[mi_edge[1]]=mi_edge[0]
            open.append(mi_edge[1])
            edge.append(mi_edge)
            for i in range(0,mi_edge[1]):
                mat[i][mi_edge[1]]+=visited[mi_edge[0]]+math.sqrt(tier[mi_edge[1]])*travel_range*punish
                # real_mat[i][mi_edge[1]] += visited[mi_edge[0]]
            for i in range(mi_edge[1]+1,len(mat)):
                mat[mi_edge[1]][i]+=visited[mi_edge[0]]+math.sqrt(tier[mi_edge[1]])*travel_range*punish
                # real_mat[mi_edge[1]][i] += visited[mi_edge[0]]
            # print(len(edge))
            # print(visited)
            # print(all(visited))
        return edge,union_tree,tier
    edge,union_tree,tier=prim(picked_sites_dis,punish)
    print("MST created")
    return edge,union_tree,tier
def site_interpolation(edges,selected_points):
    global travel_range
    edges_fine=[]
    selected_points_fine=copy.copy(selected_points)
    for edge in edges:
        start=selected_points[edge[0]]
        end=selected_points[edge[1]]
        st.reset()
        length,path=pa.find(start,end,0,[])
        if min(length,st.global_min)<=travel_range:
            edges_fine.append(edge)
        while min(length,st.global_min)>travel_range:
            print(start,'\n',end)
            reach=get_P(point_coverage(start,mode='travel'))
            print('covered')
            line=ge.LineString([start,end])
            new_site=reach.exterior.intersection(line)
            selected_points_fine.append(new_site)
            edges_fine.append([edge[0],len(selected_points_fine)-1])
            edges_fine.append([len(selected_points_fine)-1,edge[1]])
            start=new_site
            st.reset()
            length,path = pa.find(start, end, 0, [])
            print('interpolation')
    print('interpolation done')
    return  edges_fine,selected_points_fine
def find_site_dis_upper(sites_dis,union_tree,tier):
    # print(union_tree)
    res=[[-1 for _ in range(len(sites_dis[0]))] for _ in range(len(sites_dis))]
    for i in range(0,len(res)-1):
        for j in range(i+1,len(res)):
            # print('cal',i,j)
            dis=0
            # counter=0
            if tier[i]>=tier[j]:
                ma,mi=i,j
            else:
                ma,mi=j,i
            ma_tier, mi_tier = tier[ma], tier[mi]
            while ma_tier>mi_tier:
                pre=union_tree[ma]
                dis+=sites_dis[min(pre,ma)][max(pre,ma)]
                ma=pre
                ma_tier-=1
            while ma!=mi:
                # counter+=1
                # if counter==200:
                #     return
                # print(ma,mi)
                pre = union_tree[ma]
                dis += sites_dis[min(pre, ma)][max(pre, ma)]
                ma = pre
                ma_tier -= 1
                pre = union_tree[mi]
                dis += sites_dis[min(pre, mi)][max(pre, mi)]
                mi = pre
                mi_tier -= 1
            res[i][j]=dis
    print('site_dis upper_bound calculated')
    return res
def PandC(greed_flag=0,maxiters_set=20,show_sites=0,show_connection=0,show_connections_pre=0,punish=0.1):
    # global  selected_points_raw
    global selected_points_raw
    selected_points_raw=pick_sites(greed_flag,maxiters_set,show_sites)
    sites_dis=picked_sites_dis(selected_points_raw)#lower bound of real path
    raw_edges,union_tree,tier=picked_sites_MST(sites_dis,punish)
    # print(raw_edges)
    sites_dis_upper=find_site_dis_upper(sites_dis,union_tree,tier)
    if show_connections_pre:
        edges_poly=[]
        for edge in raw_edges:
            line=ge.LineString([selected_points_raw[edge[0]],selected_points_raw[edge[1]]])
            edges_poly.append(line)
        p([st.obs,pan.GeoSeries(selected_points_raw),pan.GeoSeries(edges_poly)], ['red', 'black'])
    if show_connection:
        interpolated_edges, selected_points_raw = site_interpolation(raw_edges, selected_points_raw)
        edges_poly=[]
        for edge in interpolated_edges:
            line=ge.LineString([selected_points_raw[edge[0]],selected_points_raw[edge[1]]])
            edges_poly.append(line)
        p([st.obs,pan.GeoSeries(selected_points_raw),pan.GeoSeries(edges_poly)], ['red', 'black'])
    return sites_dis,sites_dis_upper
def get_site_distance_guess(lower,upper): #get proper guess of travel distance between site
    res=[[-1 for _ in range(len(lower))] for _ in range(len(lower)) ]
    for i in range(len(lower)-1):
        for j in range(i+1,len(lower)):
            res[i][j]=min((lower[i][j]+upper[i][j])/2,lower[i][j]*1.5) # a guess of path length between sites
    return res
def cal_travel_cost(dis,payload): #calculate energy cost of this trip
# 0.05=5km 1=100km
# BCR=2.297(%/min*lb)*weight+3.879(%/min)
#1lb=0.453592kg
# BCR=5.06(%/min*kg)*weight+3.879(%/min)
#v=20m/s=1.2km/min=1.2/100/min
# BCR=422(%/kg)*weight+323.25(%)
    return (4.22*payload+3.23)*dis
def task_generator(number,k): #generate randow task
    print(number,k)
    return random.sample(range(1,int(number)),min(int(k),int(number)-1))
def branch_and_cut(dis_mat,task_number,solver='exact',plot=1): #generate task permutation find pareto optimality solution
    task=task_generator(len(dis_mat),task_number)
    def cal_fitness(dis,en,a=0.1):
        return a*en+(1-a)*dis
    def exact_solver(dis_mat,task):
        min_dis_all=float('inf')
        min_en_all=float('inf')
        max_fitness=float('inf')
        solution=[[],[],[]]
        for this_task in itertools.permutations(task):
            this_task=[0]+list(this_task)+[0]
            dis_all=0
            en_all=0
            for i in range(0,len(this_task)-1):
                mi,ma=min(this_task[i],this_task[i+1]),max(this_task[i],this_task[i+1])
                dis_all+=dis_mat[mi][ma]
                en_all+=cal_travel_cost(dis_mat[mi][ma],10*((len(this_task)-2-i)/(len(this_task)-2)))
                fitness=cal_fitness(dis=dis_all,en=en_all)
                if dis_all<min_dis_all:
                    min_dis_all=dis_all
                    solution[0]=[this_task,dis_all*100,en_all,fitness]
                if en_all<min_en_all:
                    min_en_all=en_all
                    solution[1]=[this_task,dis_all*100,en_all,fitness]
                if fitness<max_fitness:
                    max_fitness=fitness
                    solution[2]=[this_task,dis_all*100,en_all,fitness]
        return solution
    if solver=='exact':
        solution=exact_solver(dis_mat,task)
        print(solution)
    with open("D:/UAV project/solution.txt", "w+") as f:
        f.write('total_dis(km),total_energy_consumption(battery),fitness\n')
        f.write('min time solution\n')
        f.write(str(solution[0]))
        f.write('min energy consumption_solution\n')
        f.write(str(solution[1]))
        f.write('max fitness solution\n')
        f.write(str(solution[2]))
    print('task planning done')
    if plot:
        edges=[]
        task_route=solution[plot-1][0]
        for i in range(len(task_route)-1):
            st.reset()
            lenght,path=pa.find(selected_points_raw[task_route[i]],selected_points_raw[task_route[i+1]],0,[])
            if st.global_path:
                edges.extend(st.global_path)
            else:
                edges.extend(path)
        p([st.obs, pan.GeoSeries(selected_points_raw), pan.GeoSeries(edges)], ['red', 'black'])
        print('route plotted')
    return
def task_planner(lower,upper,task_number,solver='exact',plot_num=1):
    dis_mat=get_site_distance_guess(lower,upper)
    branch_and_cut(dis_mat,task_number,solver,plot_num)
    print('all set')
    return
def initialize():
    #generate coverage of every potential site
    cover_mat=obs_cover()
    add_start_cover_mat=add_start_cover(cover_mat)
    csv_write(add_start_cover_mat,'obs_cover_real_addstart')
    print('sites coverage generated')
    #generate obs distance to start
    ob_dis()
    #genrate the coord of each site
    get_obs_coord()
    return
def run(show_sites=0,show_connections_pre=1,show_connection=0,punish=0.05,plot_num=1):
    dis_lower,dis_upper=PandC(show_sites=0,show_connections_pre=1,show_connection=1,punish=0.05)
    task_planner(dis_lower,dis_upper,len(dis_lower)//2,plot_num=1)
dis_lower,dis_upper=PandC(show_sites=0,show_connections_pre=1,show_connection=1,punish=0.05)
task_planner(dis_lower,dis_upper,len(dis_lower)//2,plot_num=1)
# length,edges=pa.find(st.pstart,st.pend,0,[])
# p([st.obs, pan.GeoSeries(st.global_path)], ['red'])










    

