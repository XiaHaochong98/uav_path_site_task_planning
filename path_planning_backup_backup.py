#0.05=5km
#start point always lower than des point

import geopandas as pan
import shapely.geometry as ge
import shapely.ops as op
import matplotlib.pyplot as plt
import sys
import settings as st
sys.setrecursionlimit(1500)


for i in range(2,len(st.world)):
    st.obs[i]=st.obs[i].convex_hull
st.global_min=float('inf')
st.global_path=[]
st.global_sub_min=float('inf')
st.global_sub_path=[]
def p(geos,color):
    base=geos[0].plot()
    for i, geo in enumerate(geos[1:]):
        base=geo.plot(ax=base,color=color[i])
    plt.show()
def find(start,end,last_dis,last_path):
    # global st.obs
    # global st.global_min
    # global st.global_path
    # global st.global_sub_min
    # global st.global_sub_path
    # print('start',start,'\n end',end)
    line=ge.LineString([start,end])
    dis=float('inf')
    first_ob=None
    global keep
    for ob in st.obs:
        if line.crosses(ob):
            dis_=start.distance(ob)
            if dis_<dis and (not end.intersects(ob)) and(not start.within(ob)):
                first_ob=ob
                dis=dis_
    # print('\nfirst_ob',first_ob)
    if not first_ob:
        return start.distance(end),[line]
    else:
        # dis+=last_dis
        # new_convex=pan.GeoSeries(ge.Polygon(list(first_ob.exterior.coords[:])+[(start.x,start.y),(end.x,end.y)])).convex_hull
        # new_starts_=list(new_convex.exterior.intersection(first_ob.exterior))[0]
        # print('new_starts_',new_starts_)
        # print('start',start)
        # print('new_convex',new_convex)
        # print('first_ob',first_ob)
        # new_starts=[]
        # for points in list(new_starts_):
        #     # for coord in list(points.coords):
        #         # new_starts.apst.pend(ge.Point(coord))
        #     new_starts.extend([ge.Point(coord) for coord in list(points.coords)])
            # new_starts.apst.pend(ge.Point(list(points.coords)[-1]))
        # print('pre',new_starts,'\n\n\n')
        # print('new_starts',new_starts)
        new_convex = ge.Polygon(list(first_ob.exterior.coords[:]) + [(start.x, start.y), (end.x, end.y)]).convex_hull
        new_starts = []
        ext = new_convex.exterior.coords[:]
        ext_2 = first_ob.exterior.coords[:]
        index = ext.index((end.x, end.y))
        index_start=ext.index((start.x,start.y))
        new_path=[]
        new_dis=[]
        temp=None
        for i in range(index-1,index-len(ext), -1):
            if i<0:
                i+=len(ext)
            if ext[i] in ext_2:
                new_starts.append(ge.Point(ext[i]))
                temp=i
                index_this=ext.index(ext[i])
                this_path=[]
                this_dis=0
                lo,ma=min(index_this,index_start),max(index_this,index_start)
                for j in range(lo,ma):
                    this_dis+=ge.Point(ext[j]).distance(ge.Point(ext[j+1]))
                if this_dis*2<=new_convex.length:
                    new_dis.append(this_dis)
                    for j in range(lo,ma):
                        this_path.append(ge.LineString([ge.Point(ext[j]),ge.Point(ext[j+1])]))
                    new_path.append(this_path)
                else:
                    this_dis=first_ob.length-this_dis
                    new_dis.append(this_dis)
                    for j in range(ma,len(ext)-1):
                        this_path.append(ge.LineString([ge.Point(ext[j]), ge.Point(ext[j + 1])]))
                    this_path.append(ge.LineString([ge.Point(ext[-1]), ge.Point(ext[0])]))
                    for j in range(0,lo):
                        this_path.append(ge.LineString([ge.Point(ext[j]), ge.Point(ext[j + 1])]))
                    new_path.append(this_path)
                break
        for i in range(index+1,index+len(ext)):
            if i >= len(ext):
                i -= len(ext)
            if ext[i] in ext_2:
                if i!=temp:
                    new_starts.append(ge.Point(ext[i]))
                    index_this = ext.index(ext[i])
                    this_path = []
                    this_dis = 0
                    lo, ma = min(index_this, index_start), max(index_this, index_start)
                    for j in range(lo, ma):
                        this_dis += ge.Point(ext[j]).distance(ge.Point(ext[j + 1]))
                    if this_dis * 2 <= new_convex.length:
                        new_dis.append(this_dis)
                        for j in range(lo, ma):
                            this_path.append(ge.LineString([ge.Point(ext[j]), ge.Point(ext[j + 1])]))
                        new_path.append(this_path)
                    else:
                        this_dis = first_ob.length - this_dis
                        new_dis.append(this_dis)
                        for j in range(ma, len(ext) - 1):
                            this_path.append(ge.LineString([ge.Point(ext[j]), ge.Point(ext[j + 1])]))
                        this_path.append(ge.LineString([ge.Point(ext[-1]), ge.Point(ext[0])]))
                        for j in range(0, lo):
                            this_path.append(ge.LineString([ge.Point(ext[j]), ge.Point(ext[j + 1])]))
                        new_path.append(this_path)
                break
        next_path_real=None
        for i,new_start in enumerate(new_starts):
            st.global_sub_min = new_dis[i]
            st.global_sub_path = new_path[i]
            for ob in st.obs:
                flag=0
                for path in new_path[i]:
                    if path.crosses(ob):
                        flag=1
                        break
                if flag:
                    dis_new = start.distance(ob)
                    if dis_new!=dis:
                        st.global_sub_min = float('inf')
                        st.global_sub_path = []
                        # print(start,new_start)
                        find_subpath(start, new_start, 0, [], dis)
                        break
            # print(st.global_sub_path,st.global_sub_min)
            dis_pre=last_dis+st.global_sub_min
            path_pre=last_path+st.global_sub_path
            if dis_pre+new_start.distance(end)<st.global_min-0.03:#can add an offset someproblem in this prun,should put in more argument
                # print(new_start,end)
                keep=[new_start,end,first_ob,new_convex]
                # print(end)
                next_dis,next_path=find(new_start,end,dis_pre,path_pre)#error! should check (start new_start) cross st.obstacle
                if dis_pre+next_dis<st.global_min:
                    st.global_min=dis_pre+next_dis
                    st.global_path=path_pre+next_path
                    next_path_real=next_path
                    next_start=new_start
        if not next_path_real:
            return float('inf'),[]
        else:
            return st.global_min-last_dis,[ge.LineString([start,next_start])]+next_path_real

keep=None
def find_subpath(start,end,last_dis,last_path,ob_dis):
    # global st.obs
    # global st.global_sub_min
    # global st.global_sub_path
    global keep
    line = ge.LineString([start, end])
    dis = float('inf')
    first_ob = None
    for ob in st.obs:
        if line.crosses(ob):
            dis_=start.distance(ob)
            if dis_==ob_dis:
                if end.intersects(ob):
                    sumdis=0
                    ob_path=[]
                    ext_ob=ob.exterior.coords[:]
                    for i in range(len(ext_ob)):
                        if ext_ob[i]==start.coords:
                            start=i
                            p2=ge.Point(ext_ob[i])
                            while ext_ob[i]!=end.coords:
                                i+=1
                                p1=p2
                                p2=ge.Point(ext_ob[i])
                                sumdis+=p1.distance(ge.Point(p2))
                                ob_path.append(ge.LineString([p1,p2]))
                            end=i
                            if ob.length<2*sumdis:
                                sumdis=ob.length-sumdis
                                ob_path=[]
                                for j in range(start,0,-1):
                                    ob_path.append(ge.LineString([ext_ob[i],ext_ob[i-1]]))
                                for j in range(len(ext_ob)-1,end,-1):
                                    ob_path.append(ge.LineString([ext_ob[i], ext_ob[i-1]]))
                            # print('return ob_path',ob_path)
                            return sumdis,ob_path

            if dis_ < dis and dis_!=ob_dis and (not end.within(ob)):
                # print('in',dis,ob_dis)
                first_ob = ob
                dis = dis_
    # print('\nfirst_ob',first_ob)
    if not first_ob:
        return start.distance(end), [line]
    else:
        # dis+=last_dis
        new_convex =ge.Polygon(list(first_ob.exterior.coords[:]) + [(start.x, start.y), (end.x, end.y)]).convex_hull
        # new_starts_ = list(new_convex.exterior.intersection(first_ob.exterior))[0]
        # print('new_starts_',new_starts_)
        # print('start',start)
        # print('new_convex',new_convex)
        # print('first_ob',first_ob)
        new_starts = []
        ext=new_convex.exterior.coords[:]
        ext_2=first_ob.exterior.coords[:]
        index=ext.index((end.x,end.y))
        temp = None
        for i in range(index - 1, index - len(ext), -1):
            if i < 0:
                i += len(ext)
            if ext[i] in ext_2:
                new_starts.append(ge.Point(ext[i]))
                temp = i
                break
        for i in range(index + 1, index + len(ext)):
            if i >= len(ext):
                i -= len(ext)
            if ext[i] in ext_2:
                if i != temp:
                    new_starts.append(ge.Point(ext[i]))
                break
        next_path_real = None
        for new_start in new_starts:
            # print(line,'\n',first_ob,'\n',new_convex,'\n',new_starts,'\n',dis,'\n',ob_dis,'\n',first_ob,'\n\n')
            this_dis, this_path = find_subpath(start, new_start,0,[],dis)
            # print(this_dis)
            dis_pre = last_dis + this_dis
            path_pre = last_path + this_path
            if dis_pre + new_start.distance(
                    end) < st.global_sub_min - 0.005:  # can add an offset someproblem in this prun,should put in more argument
                # print('sub',new_start,start)
                next_dis, next_path = find_subpath(new_start, end, dis_pre,
                                           path_pre,float('inf'))  # error! should check (start new_start) cross st.obstacle
                if dis_pre + next_dis < st.global_sub_min:
                    st.global_sub_min = dis_pre + next_dis
                    st.global_sub_path = path_pre + next_path
                    next_path_real = next_path
                    next_start = new_start
        if not next_path_real:
            return float('inf'), []
        else:
            return st.global_sub_min - last_dis, [ge.LineString([start, next_start])] + next_path_real


#example
def pp(po):
    p([st.world,pan.GeoSeries(po)],['red'])
# len,path=find(st.pstart,st.pend,0,[])
# p([st.world,pan.GeoSeries(st.global_path)],['red'])
# print(st.global_min)
# POINT (116.3409233093262 39.98093477655269) POINT (116.4109233093262 39.98093477655269)
# len,path=find(ge.Point(116.3409233093262,39.98093477655269),ge.Point(116.488,40.0608),0,[])
# p([st.world,pan.GeoSeries(st.global_path)],['red'])
# print(len)
# p([st.world,pan.GeoSeries([ge.Point(116.3409233093262,39.98093477655269),ge.Point(116.4109233093262,39.98093477655269)])],['red'])

