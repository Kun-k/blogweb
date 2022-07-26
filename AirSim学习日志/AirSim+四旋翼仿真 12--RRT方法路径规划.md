之前已经实现了人工势场法避障的python仿真，人工势场法适用于局部避障，不依赖全局障碍物信息，根据实时检测到的障碍物即可进行避障。但其不能确保得到的路径最优，且存在局部极小值等问题。如果在已知部分障碍物信息的情况下，进行全局的路径规划，以局部避障方法作为辅助，可以得到更好的效果。

经过算法调研，了解到RRT方法（快速扩展随机树）和PRM方法（概率路线图方法）可以实现全局障碍物信息下的路径规划。PRM方法为，在地图中进行点采样，通过采样点获取可行路径；RRT方法为，从起始点开始进行节点扩展，每次有一定概率向目标点或一个随机点扩展，最终生成一棵包含可行路径的树。可通过A*等算法在当前采样点中寻找最优路径。

算法原理比较简单，接下来说明算法实现步骤，详细的代码见https://github.com/Kun-k/airsim_python/blob/main/code_python/python_avoid_RRT.py。

#### 1.初始化地图

取一个二维数组作为输入地图，0表示该点为障碍物，1表示该点为可行点。读取一张图片并二值化，即可得到一个这样的数组。

```python
# 读取地图图像并二值化
img = cv2.imread('6.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
retval, mymap = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
mymap = cv2.dilate(mymap, None, iterations=1).T
mymap = cv2.erode(mymap, None, iterations=4) / 255
```

为了便于记录和去重，生成一个与`mymap`相同尺寸的数组，用于储存各个位置的树节点。

```python
tree_map = []  # 记录不同位置产生的节点，用于生成树时去重，0表示障碍物，None表示未访问，否则为已访问
for i in range(mapsize[0]):
    tree_map.append([])
    for j in range(mapsize[1]):
        if mymap[i][j] == 0:
            tree_map[i].append(0)
        else:
            tree_map[i].append(None)
```

#### 2.设置航路点和参数

取下图所示的地图，分辨率为`600x750`，出发航路点为左下的`[0,0]`点，目标航路点为右上的`[380,700]`航路点。

<img src="https://cdn.jsdelivr.net/gh/kun-k/blogweb/image6-flip.png" alt="6-flip" style="zoom:50%;" />

可设置的参数包括：

1. 随机采样概率`p_sample`，在`p_sample`的概率下选择一个随机点作为采样点，否则以目标点作为采样点；
2. 最大迭代次数`maxlimit`，若超出该迭代次数后仍未查找到路径，则认为查找失败，退出；
3. 步长`step`，即每次向采样点移动的距离；
4. 误差`epsilon`，与目标点的距离小于该值时认为查找到路径；
5. 安全距离`Q_save`，选择航路点时，与障碍物的距离需大于安全距离。

#### 3.点采样

在`p_sample`的概率下选择一个随机点作为采样点，否则以目标点作为采样点，

```python
# 获取采样点
sample = [0, 0]
while tree_map[sample[0]][sample[1]] is not None:
    if np.random.rand() < p_sample:
        sample = np.array([np.random.randint(0, mapsize[0]),
                           np.random.randint(0, mapsize[1])])
    else:
        sample = aim
```

#### 4.树的生长

获取采样点后，在已搜索的点中，选择距离采样点最近的点作为当前的出发节点`p_curr`，

```python
# 在已搜索的点中，寻找距离采样点最近的点p_curr
min_dis = float('inf')
p_curr = start
for p_tmp in p_record:
    dis = distance(p_tmp, sample)
    if dis < min_dis:
        p_curr = p_tmp
        min_dis = dis
```

接下来由`p_curr`出发向采样点生长，生长的步长为`step`，得到点`p_next`

```python
direction = (sample - p_curr) / np.linalg.norm((sample - p_curr))
p_next = p_curr + direction * step
```

需判断点`p_next`是否可行，这里进行三个限制

1. `p_next`位置无障碍物

   ```python
   if 0 <= p_next[0] < mapsize[0] and 0 <= p_next[1] < mapsize[1] \
           and tree_map[int(p_next[0])][int(p_next[1])] is None:  # 是否为非障碍物点
       flag = True  # True表示该点可行
   ```

2. `p_next`周围安全距离的范围内无障碍物

   ```python
   for dir in dir_save:  # 判断点是否为安全点
       p_search = p_next + dir
       if not (0 <= p_search[0] < mapsize[0] and 0 <= p_search[1] < mapsize[1]
               and tree_map[int(p_search[0])][int(p_search[1])] != 0):
           flag = False
           break
   ```

3. `p_curr`到`p_next`的路径上是否存在障碍物

   ```python
   d_next = distance(p_curr, p_next)
   direction = (p_next - p_curr) / np.linalg.norm((p_next - p_curr))
   for d_search in range(1, int(d_next) + 1):
       p_search = p_curr + d_search * direction
       if not (0 <= p_search[0] < mapsize[0] and 0 <= p_search[1] < mapsize[1]
               and tree_map[int(p_search[0])][int(p_search[1])] != 0):  # 是否为安全点
           flag = False
           break
   ```

如果同时满足这三个条件，则认为点`p_next`可行，`p_curr`到`p_next`的路径可行，此时在`p_next`产生一个新节点，其父节点为`p_curr`，

```python
parenttree = tree_map[int(p_curr[0])][int(p_curr[1])]
newtree = rtt_treenode(p_next, parent=parenttree,
                       cost_from_start=parenttree.cost + step,
                       dis_to_aim=distance(p_next, aim))
tree_map[int(p_next[0])][int(p_next[1])] = newtree
parenttree.children.append(newtree)
p_record.append(p_next)
e = newtree.dis
```

#### 5.树生长的终止条件

当查找到一个节点，其与目标点的距离小于`epsilon`，则终止；或迭代次数超出设定值，终止。

#### 6.最优路径搜索

这里使用A*算法进行搜索。

```python
def A_star(root, eposilon):
    q = PriorityQueue()
    q.push(root)
    while q.size() != 0:
        node_curr = q.pop()
        for child in node_curr.children:
            # 生成树时已经去重，搜索时不再进行
            if child.dis < eposilon:  # 目标点
                return child.path()
            q.push(child)
    return []
```

#### 7.结果展示

设置参数为

```python
p_sample=0.1, maxlimit=5000, step=100, eposilon=50, Q_save=10
```

得到的结果为：

![屏幕捕获_2022_07_22_12_03_40_89](https://cdn.jsdelivr.net/gh/kun-k/blogweb/image屏幕捕获_2022_07_22_12_03_40_89.png)

在UE中设置障碍物，上图中的10个像素点对应UE中的1米，即步长为10米，误差为5米，安全距离为1米。将上图中得到的路径点调整为AirSim中的航路点后，使用之前完成的航路点跟踪算法，得到的飞行效果如下：

（航路点跟踪算法见 https://blog.csdn.net/k_kun/article/details/125726077?spm=1001.2014.3001.5501）

<video src="D:\User_D\Documents\oCam\录制_2022_07_22_12_04_55_683.mp4"></video>

设置不同的参数，将得到不同的效果。将采样概率修改为0.5后，得到树的随机性变大，分枝数变多：

![image-20220726115721513](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220726115721513.png)

继续修改参数，将步长调整为20后，得到的树节点更加密集，计算量也更大。此时，AirSim仿真中两个航路点的距离为2米，距离过短，在较快速度下的飞行效果比较差。

![image-20220726120122822](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220726120122822.png)





RRT方法参考来源：https://zhuanlan.zhihu.com/p/66047152