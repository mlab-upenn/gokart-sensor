Draw = True
import numpy as np
import os

if Draw:
    import seaborn as sns
    import matplotlib.pyplot as plt

def cubic(start,end,*args):
    count=4*(len(args)-1)
    mat_ori=np.zeros((count,count))
    mat_ans=np.zeros(count)
    index=0
    for i,j in zip(args[:-2],args[1:-1]):
        mat_ori[4*index,4*index]=i[0]**3
        mat_ori[4*index,4*index+1]=i[0]**2
        mat_ori[4*index,4*index+2]=i[0]
        mat_ori[4*index,4*index+3]=1
        mat_ans[4*index]=i[1]
        mat_ori[4*index+1,4*index]=j[0]**3
        mat_ori[4*index+1,4*index+1]=j[0]**2
        mat_ori[4*index+1,4*index+2]=j[0]
        mat_ori[4*index+1,4*index+3]=1
        mat_ans[4*index+1]=j[1]
        mat_ori[4*index+2,4*index]=3*j[0]**2
        mat_ori[4*index+2,4*index+1]=2*j[0]
        mat_ori[4*index+2,4*index+2]=1
        mat_ori[4*index+2,4*index+4]=-3*j[0]**2
        mat_ori[4*index+2,4*index+5]=-2*j[0]
        mat_ori[4*index+2,4*index+6]=-1
        mat_ans[4*index+2]=0
        mat_ori[4*index+3,4*index]=6*j[0]
        mat_ori[4*index+3,4*index+1]=2
        mat_ori[4*index+3,4*index+4]=-6*j[0]
        mat_ori[4*index+3,4*index+5]=-2
        mat_ans[4*index+3]=0
        index+=1
    mat_ori[4*index,4*index]=args[-2][0]**3
    mat_ori[4*index,4*index+1]=args[-2][0]**2
    mat_ori[4*index,4*index+2]=args[-2][0]
    mat_ori[4*index,4*index+3]=1
    mat_ans[4*index]=args[-2][1]
    mat_ori[4*index+1,4*index]=args[-1][0]**3
    mat_ori[4*index+1,4*index+1]=args[-1][0]**2
    mat_ori[4*index+1,4*index+2]=args[-1][0]
    mat_ori[4*index+1,4*index+3]=1
    mat_ans[4*index+1]=args[-1][1]
    mat_ori[4*index+2,0]=3*args[0][0]**2
    mat_ori[4*index+2,1]=2*args[0][0]
    mat_ori[4*index+2,2]=1
    mat_ans[4*index+2]=start
    mat_ori[4*index+3,4*index]=3*args[-1][0]**2
    mat_ori[4*index+3,4*index+1]=2*args[-1][0]
    mat_ori[4*index+3,4*index+2]=1
    mat_ans[4*index+3]=end
    mat_rg=np.linalg.solve(mat_ori,mat_ans)
    def rtn_func(x):
        def bin_search(left,right):
            if(x<=args[left][0]):return left
            if(x>=args[right-1][0]):return right-1
            if(right-left<=1):return left
            mid=int((left+right)/2)
            if(x<args[mid][0]):return bin_search(left,mid)
            else:return bin_search(mid,right)
        num=bin_search(0,len(args)-1)
        return mat_rg[4*num]*x**3+mat_rg[4*num+1]*x**2+mat_rg[4*num+2]*x+mat_rg[4*num+3]
    return rtn_func

left_up = np.array([9.65, 8.52])  # (x2, y2)
right_down = np.array([-13.7, -0.1])  # (x1, y1)
y1, y2 = right_down[1], left_up[1]  # y1 < y2
x1, x2 = right_down[0], left_up[0]  # x1 < x2


outer_rect = [10.5, 25]
inner_rect = [7, 22]
"""
     l1
  ________
  |  l2  |
  |  __  |
l3| |  | |
  | |__| |
  |______|
(0, 0)      
"""
l1, l3 = outer_rect[0], outer_rect[1]
l2, l4 = inner_rect[0], inner_rect[1]
corner_rect = [(l1-l2)/2, (l3-l4)/2]   #  (h, v)
h, v = corner_rect[0], corner_rect[1]
print(h, v)

arcOffsetLongV = 1.2
arcOffsetLongH = 0.15
arcOffsetShortH = 0.5
arcOffsetShortV = 0.1
arcOffsetCurve = 0.5
cubic_k = 7
arcPointsNum = 15
LongPointsNum = 40
ShortPointsNum = 20

# 
left_certerP = [h/2 - arcOffsetLongH , l3/2]
right_certerP = [l1, l3/2]
upper_certerP = [l1/2, l3]
bottom_certerP = [l1/2, v/2 - arcOffsetShortV]

# for arc located in (0, 0) 
arc_centerTheta = np.arctan2(v, h)
arc_certerP = [h/2 + arcOffsetCurve * np.cos(arc_centerTheta), v/2 + arcOffsetCurve * np.sin(arc_centerTheta)]
arcLongP = [h/2 - arcOffsetLongH, v+arcOffsetLongV]
arcShortP = [h+arcOffsetShortH, v/2 - arcOffsetShortV]
data = [arcLongP, arc_certerP, arcShortP]
func1=cubic(-cubic_k,0,*data)

# get waypoints
arcLeftBottom_x = np.linspace(h/2 - arcOffsetLongH, h+arcOffsetShortH, arcPointsNum)[::-1]
arcLeftBottom_y = np.array([func1(i) for i in arcLeftBottom_x])
half_left_y = np.linspace(v+arcOffsetLongV, left_certerP[1], int(LongPointsNum/2))
half_left_x = np.ones_like(half_left_y) * left_certerP[0]
half_bottom_x = np.linspace(h+arcOffsetShortH, bottom_certerP[0], int(ShortPointsNum/2))[::-1]
half_bottom_y = np.ones_like(half_bottom_x) * bottom_certerP[1]
half_left_bottom = np.hstack([np.vstack([half_bottom_x, half_bottom_y]) , np.vstack([arcLeftBottom_x, arcLeftBottom_y]), np.vstack([half_left_x, half_left_y])])  # (2, n)
half_left_up_x = half_left_bottom[0][::-1]
half_left_up_y = (-half_left_bottom[1]+2*left_certerP[1])[::-1]
half_left_up = np.vstack([half_left_up_x, half_left_up_y])

left = np.hstack([half_left_bottom, half_left_up])
right_y = left[1][::-1]
right_x = -left[0] + 2*bottom_certerP[0]
right = np.vstack([right_x, right_y])
waypoints = np.hstack([left, right])  # (2, n)

if Draw:
    fig = plt.figure(figsize=(16, 9))
    plt.axis('equal')
    plt.plot([0, 0, l1, l1, 0], [0, l3, l3, 0, 0], '-b', linewidth=1.0)

    plt.plot([h, h, v+l2, v+l2, h], [v, l4+h, l4+h, v, v], '-r', linewidth=1.0)

    # plt.plot([h, h+l2], [v, v],'-r')

    # plt.plot(left[0],left[1], 'or', markersize=1.0)
    # plt.plot(right[0],right[1], 'or', markersize=1.0)
    plt.plot(waypoints[0], waypoints[1], 'or', markersize=1.0)
    # plt.plot(x,arcPointsLeftBottom_y, '-or')
    plt.plot([i[0] for i in data],[i[1] for i in data],'ob', markersize=1.0)
    plt.show()

# change_for car
home = '/sim_ws'
origin = np.array([-4.67, 0.673]).reshape(2, 1)   # (2, 1)

theta = -np.pi/2 + 2*np.pi/180
R = np.array(
    [[np.cos(theta), -np.sin(theta)],
    [np.sin(theta), np.cos(theta)]]
)
waypoints = R @ waypoints + origin

def floorUp(x):
    if np.floor(2*x) > 2*np.floor(x):
        return (np.floor(x) + 1).astype(np.int32)
    else:
        return np.floor(x).astype(np.int32)

# interp speed
sOffset = 0.8
speed_max = 4.8
speed_min = 2.0
shortBufferOffset = 4
longBufferOffset = 5
short_buffer = floorUp(2*arcPointsNum/3 + shortBufferOffset)
long_buffer = floorUp(arcPointsNum/3 + longBufferOffset)
print(f'short_buffer: {short_buffer}, long_buffer{long_buffer}')

v = np.array([-1])
v = np.hstack((v, np.array([speed_max-sOffset]*int(ShortPointsNum/2-shortBufferOffset))))
v = np.hstack([v, np.linspace(speed_max-sOffset, speed_min, short_buffer)])
v = np.hstack([v, np.linspace(speed_min, speed_max, long_buffer)])  
# print(len(v)-1)   

v = np.hstack([v, np.array([speed_max]*int(LongPointsNum-2*longBufferOffset +1))])
v = np.hstack([v, np.linspace(speed_max, speed_min, long_buffer)])
v = np.hstack([v, np.linspace(speed_min, speed_max-sOffset, short_buffer-1)])
# print(len(v)-1)  

v = np.hstack([v, np.array([speed_max-sOffset]*int(ShortPointsNum-2*shortBufferOffset +1))])
v = np.hstack([v, np.linspace(speed_max-sOffset, speed_min, short_buffer)])
v = np.hstack([v, np.linspace(speed_min, speed_max, long_buffer)])
# print(len(v)-1)  

v = np.hstack([v, np.array([speed_max]*int(LongPointsNum-2*longBufferOffset))])
v = np.hstack([v, np.linspace(speed_max, speed_min, long_buffer)])
v = np.hstack([v, np.linspace(speed_min, speed_max-sOffset, short_buffer-1)])

v = np.hstack((v, np.array([speed_max-sOffset]*int(ShortPointsNum/2-shortBufferOffset))))
# print(len(v)-1)
v = v[1:]

if Draw:
    n = len(v)
    sns.scatterplot(x=waypoints[0, :n], y=waypoints[1, :n],
                    hue=v)
    plt.show()

else:
    wp_file = open(os.path.join(home+'/wp_log', 'wp1_interp.csv'), 'w')
    for x, y, v in zip(waypoints[0], waypoints[1], v):
        wp_file.write('%f, %f, %f\n' % (x, y, v))
    wp_file.write('%f, %f, %f\n' % (arcPointsNum, LongPointsNum, ShortPointsNum))
    wp_file.close()
