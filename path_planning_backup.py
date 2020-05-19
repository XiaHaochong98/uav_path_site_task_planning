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
def find(start,end,last_dis,last_path,is_main=1):
    # print('start', start, '\n', 'end', end)
    if start.equals(end):
        if is_main:
            st.global_path=[]
            st.global_min=0
        return 0,[]
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
        if is_main:
            st.global_path=[line]
            st.global_min=start.distance(end)
        return start.distance(end),[line]
    else:
        new_convex = ge.Polygon(list(first_ob.exterior.coords[:]) + [(start.x, start.y), (end.x, end.y)]).convex_hull
        new_starts = []
        ext = new_convex.exterior.coords[:]
        ext_2 = first_ob.exterior.coords[:]
        index = ext.index((end.x, end.y))
        index_start=ext.index((start.x,start.y))
        new_path=[]
        new_dis=[]
        temp=None
        # keep=ext
        # print(index-1,ext)
        # print('type',type(ext))
        # print('len',len(ext))
        for i in range(index-1,index-len(ext),-1):
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
                    this_dis=new_convex.length-this_dis
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
                        this_dis = new_convex.length - this_dis
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
                        temp_dis,temp_path=find_subpath(start, new_start, 0, [], dis)
                        # print('out')
                        # st.global_min=min(st.global_sub_min,temp_dis)
                        # if not st.global_sub_path:
                            # st.global_sub_path=temp_path
                        break
            # print('now',st.global_sub_path,st.global_sub_min)
            # print('st.glo'st.global_sub_path,)
            dis_pre=last_dis+st.global_sub_min
            path_pre=last_path+st.global_sub_path
            if dis_pre+new_start.distance(end)<st.global_min-0.03:#can add an offset someproblem in this prun,should put in more argument
                # print(new_start,end)
                keep=[new_start,end,first_ob,new_convex]
                # print(end)
                # p([st.world, pan.GeoSeries(st.global_sub_path)], ['red'])
                # p([st.world, pan.GeoSeries(last_path)], ['red'])
                next_dis,next_path=find(new_start,end,dis_pre,path_pre,is_main=0)#error! should check (start new_start) cross st.obstacle
                if dis_pre+next_dis<st.global_min:
                    st.global_min=dis_pre+next_dis
                    # print('path',start,new_start,st.global_sub_path)
                    # p([st.world, pan.GeoSeries(st.global_sub_path)], ['red'])
                    # p([st.world, pan.GeoSeries(last_path)], ['red'])
                    # p([st.world, pan.GeoSeries(next_path)], ['red'])
                    st.global_path=path_pre+next_path
                    next_path_real=next_path
                    next_start=new_start
        if not next_path_real:
            return float('inf'),[]
        else:
            return st.global_min-last_dis,[ge.LineString([start,next_start])]+next_path_real

keep=None
def find_subpath(start,end,last_dis,last_path,ob_dis,is_main=1):
    # global st.obs
    # global st.global_sub_min
    # global st.global_sub_path
    # global keep
    if start.equals(end):
        if is_main:
            st.global_sub_path=[]
            st.global_sub_min=0
        return 0,[]
    line = ge.LineString([start, end])
    dis = float('inf')
    first_ob = None
    # print('start',start,'\n','end',end)
    for ob in st.obs:
        if line.crosses(ob):
            # print('cross')
            dis_=start.distance(ob)
            # print(dis_,ob_dis)
            if dis_==ob_dis:
                new_convex = ge.Polygon(
                    list(ob.exterior.coords[:]) + [(start.x, start.y), (end.x, end.y)]).convex_hull
                ext = new_convex.exterior.coords[:]
                index_end = ext.index((end.x, end.y))
                index_start = ext.index((start.x, start.y))
                sumdis=0
                ob_path=[]
                mi,ma=min(index_start,index_end),max(index_start,index_end)
                for j in range(mi,ma):
                    sumdis+=ge.Point(ext[j]).distance(ge.Point(ext[j+1]))
                if sumdis*2<=new_convex.length:
                    for j in range(mi,ma):
                        ob_path.append(ge.LineString([ge.Point(ext[j]),ge.Point(ext[j+1])]))
                else:
                    sumdis=new_convex.length-sumdis
                    for j in range(ma,len(ext)-1):
                        ob_path.append(ge.LineString([ge.Point(ext[j]), ge.Point(ext[j + 1])]))
                    ob_path.append(ge.LineString([ge.Point(ext[-1]), ge.Point(ext[0])]))
                    for j in range(0,mi):
                        ob_path.append(ge.LineString([ge.Point(ext[j]), ge.Point(ext[j + 1])]))
                # print('sumdis',sumdis,'\n','ob_path',ob_path)
                return sumdis,ob_path
            if dis_!=ob_dis: #and dis_ < dis  and (not end.within(ob)):
                # print('in',dis,ob_dis)
                first_ob = ob
                dis = dis_
    # print('\nfirst_ob',first_ob)
    if not first_ob:
        # print('no ob')
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
            this_dis, this_path = find_subpath(start, new_start,0,[],dis,is_main=0)
            # print('this',this_path)
            dis_pre = last_dis + this_dis
            path_pre = last_path + this_path
            if is_main:
                if dis_pre + new_start.distance(
                        end) < st.global_sub_min - 0.005:  # can add an offset someproblem in this prun,should put in more argument
                    # print('sub',new_start,start)
                    # print(new_start,end)
                    # print('last',last_path)
                    # print('this',this_path)
                    # p([st.world, pan.GeoSeries(last_path)], ['red'])
                    # p([st.world, pan.GeoSeries(this_path)], ['red'])
                    # print(new_start,end)
                    next_dis, next_path = find_subpath(new_start, end, dis_pre,
                                               path_pre,float('inf'))  # error! should check (start new_start) cross st.obstacle
                    # print('next_path',next_path)
                    # p([st.world, pan.GeoSeries(next_path)], ['red'])
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
# length,path=find(st.pstart,st.pend,0,[])
# p([st.world,pan.GeoSeries(st.global_path)],['red'])
# print(st.global_min)
# POINT (116.3409233093262 39.98093477655269) POINT (116.4109233093262 39.98093477655269)
# length,path=find(ge.Point(116.3409233093262,39.98093477655269),ge.Point(116.4319038391113,40.04952971114968),0,[])
# length,path=find(ge.Point(116.4127206802368,40.04736149731608),ge.Point(116.3544416427612,40.0202858709469),0,[])
# print(st.global_sub_min,st.global_sub_path)
# length,path=find_subpath(ge.Point(116.4173555374145,40.02317795536186),ge.Point(116.3806200027466,39.98718249615463),0,[],0.04043142003972606)
# length,edges=find(ge.Point(116.5060615539551,40.10049568791855),ge.Point(116.5532020060342,40.05335523583944),0,[])
# length,edges=find(st.pstart,st.pend,0,[]

# p([st.world,pan.GeoSeries(st.global_path)],['red'])
# print(st.global_sub_path,st.global_sub_min)

# st.reset()
# length,edges=find(st.pstart,st.pend,0,[])
# p([st.obs, pan.GeoSeries(st.global_path)], ['red'])
# print(len)
# p([st.world,pan.GeoSeries([ge.Point(116.3409233093262,39.98093477655269),ge.Point(116.4109233093262,39.98093477655269)])],['red'])

