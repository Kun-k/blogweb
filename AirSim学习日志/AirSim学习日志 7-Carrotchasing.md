之前参考知乎教程学习了离散LQR方法的无人机轨迹跟踪，可以实现对一条确定位置、速度、角速度的轨迹进行跟踪。此外，AirSim提供了接口`moveOnPathAsync()`，通过跟踪一系列航路点实现轨迹跟踪。在网上查阅到了Carrot Chasing算法，可实现航路点跟踪的路径规划，于是阅读了相关论文并尝试在AirSim中实现该算法。

### 1.Carrot-chasing Algorithm算法

这里参考了论文Path Planning of Unmanned System using Carrot-chasing Algorithm，感兴趣的朋友可以自己下载看一下：https://arxiv.org/abs/2012.13227。这篇文章不是很长，关于算法基本原理说的比较清楚，但是对控制器的原理和调参没有太深入说明。等以后有时间了再自己探索一下。

这个算法所跟踪的是一系列航点，如下图所示，图中用`x`号标记了六个航点，按照顺序相连得到一条蓝色轨迹。要求无人机跟踪航点，实际上将问题转换为无人机跟踪这条蓝色轨迹。

![image-20220707102837497](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220707102837497.png)

首先考虑跟踪一条轨迹的情况。下图是在论文中截出来的一张示意图。

![image-20220707103330073](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220707103330073.png)

* 图中位置的定义为：

  两个航点$W_i$和$W_{i+1}$（右上的航点应为$W_{i+1}$，图中的标注错误），点$P$是无人机当前所处的位置，$S$是无人机当前的目标点。
* 图中角度的定义为：

  无人机当前的速度方向与水平方向呈角度$\psi$；无人机与目标点$S$的连线，与水平方向呈角度$\phi_d$；无人机与起始点$W_i$的连线，与水平方向呈角度$\theta_u$；两个航路点的连线，与水平方向呈角度$\theta$。
* 图中距离的定义为：

  无人机到起始点的距离$R_u$，$R_u$到目标航迹的投影$R$，无人机到目标航迹的距离$e$（即误差），无人机前向视野范围$\delta$（用于计算下一航点位置）。

算法核心为根据当前位置$P$和目标航点$S$的位置关系，调整当前速度方向，即计算角度$\phi_d$，根据其调整当前速度的方向，使无人机逼近目标轨迹。

根据已知的$W_i,W_{i+1},P$三个点的位置、角度$\psi$、视野范围$\delta$，利用几何关系，可以计算得其他所有参数。计算流程为：

1. 根据正切角关系，计算$\theta,\theta_u$，$\beta=\theta-\theta_u$；
2. 根据位置关系，计算距离$R_u$，利用$\beta$的三角函数，计算得其投影$R$和误差$e$；
3. 计算$S$到$W_i$的距离为$R+\delta$，利用$\theta$的三角函数得到$S$的位置；
4. 根据$S,P$的位置关系，计算角度$\psi_d$。

这就得到了我们需要的误差$e$和角度$\psi_d$，然后利用比例控制的方法调整当前速度，算法如下：

1. 速率固定为$V_a$，分解至$x,y$两个方向为

   $$
   v_x=v_acos\psi
   \\v_y=v_asin\psi

   $$
2. 计算控制量

   $$
   \psi'=K(\psi_d-\psi)
   \\u=\psi'·v_a

   $$

   其中$K$为控制器的比例系数。
3. 计算调整后的速度

   $$
   v_y=v_a·sin(\psi_d)+u·dt

   $$

   $dt$为计算时的单位时间。这个控制器的原理没有深究，可以简单理解为，目标角度$\psi$决定了速度希望调整至的位置，但这一目标角度是具有时效性的；两个角度的差值$\psi'$决定了速度调整的方向，在$v_y$方向增加$\psi'·v_a$有利于向目标轨迹贴近，也相当于预测了未来的速度方向。

### 2.算法实现效果

设置初始位置为$(3,3),\psi=0.5rad$，希望跟踪的轨迹为$(0,0),(10,15)$，$\delta=1,K=0.5$，实现效果如下。为避免点过于密集影响观察，图中绘制的目标航点是经过采样的：

![image-20220707115253501](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220707115253501.png)

跟踪一段轨迹的效果如下，可以看出在转角处的偏差比较大，这是由于会提前预测目标轨迹，其目标航点已经转移到了新的线段上，故轨迹会产生偏移。

![image-20220707115617311](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220707115617311.png)

如果将$\delta$设置为0.1，则其预测的距离会减小，所以会更加贴合目标轨迹，如下图所示。

![image-20220707115800874](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220707115800874.png)

理论上来说，$\delta$越小，与目标轨迹就越接近，但是实际情况下的无人机不能产生速度突变，如果设置$\delta$过小，其在转角处无法迅速改变速度方向，会产生超调。

以上算法的代码会附在文末。

### 3.在AirSim仿真环境中的效果

将2中的代码稍作调整，迁移到AirSim环境中。设置$\delta=1.5,K=0.8$，无人机初始位置为$(0,0),\psi=0$，无人机速度为1。在2中测试时没有太关心速度的大小，因为不需要考虑速度突变的问题。设置轨迹为

```python
points = [airsim.Vector3r(5, 0, -3),
          airsim.Vector3r(5, 8, -3),
          airsim.Vector3r(8, 12, -3),
          airsim.Vector3r(4, 9, -3)]
```

得到的效果如下：

![image-20220707120444373](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220707120444373.png)

可以看到实现了对航路点的跟踪，但是转角处会有明显的偏差。接下来设置$\delta=0.8$，得到的效果如下，可以看到跟踪的效果明显更好。

![image-20220707120926543](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220707120926543.png)

如果将无人机的速度调大，设置为10，这时候无人机已经完全放飞自我了，控制的速度赶不上飞机的速度，效果如下：

![image-20220707121217141](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220707121217141.png)

这部分代码也会放在文末。

### 4.Carrotchasing算法的不足

以上算法的实现是基于很多假设的，如不考虑外界环境的影响，忽略风力、气压、信号噪声等，假设无人机只跟踪$x-y$平面的轨迹等，而且只使用了比例控制器。

另外，Carrotchasing只考虑到了运动学，没有考虑到动力学，所以它不能像LQR算法那样精确地跟踪位置、速度、加速度，而且还需要假设无人机的速率是固定的。

如果以后有机会，我希望对上面实现的算法进行两方面的改进。一是尝试实现对三维情况下的航路点跟踪，二是弄清楚控制算法的原理，尝试设置复杂一点的控制器。

### 5.代码

CarrotChasing算法实现：

```python
def distance(A, B):
    return math.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)


def myatan(A, B):
    x1, y1, x2, y2 = A[0], A[1], B[0], B[1]
    if x1 != x2:
        if x1 > x2:
            return math.atan((y1 - y2) / (x1 - x2)) + math.pi
        else:
            return math.atan((y1 - y2) / (x1 - x2))
    if x1 == x2 and y1 == y2:
        return None
    if x1 == x2 and y1 != y2:
        if y1 > y2:
            return -math.pi / 2
        else:
            return math.pi / 2


def move_by_path(P, Va, psi, Path, delta=1, K=0.5, K2=0, dt=0.01):
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.rcParams["axes.unicode_minus"] = False
    [Px, Py] = P
    # Wb = P
    pos_record = [[Px, Py]]
    aim_record = []
    for i in range(1, len(Path)):
        Wa = Path[i - 1]
        Wb = Path[i]
        theta = myatan(Wa, Wb)
        while True:
            theta_u = myatan(Wa, [Px, Py])
            if theta_u == None:
                theta_u = theta
            beta = theta - theta_u
            Ru = distance(Wa, [Px, Py])
            R = Ru * math.cos(beta)
            e = Ru * math.sin(beta)
            print(Px, Py, e)
            xt = Wa[0] + (R + delta) * math.cos(theta)
            yt = Wa[1] + (R + delta) * math.sin(theta)
            if (xt - Wb[0]) * (Wb[0] - Wa[0]) > 0 \
                    or (yt - Wb[1]) * (Wb[1] - Wa[1]) > 0:
                break
            psi_d = myatan([Px, Py], [xt, yt])
            u = K * (psi_d - psi) * Va + K2 * e
            if u > 1:  # 限制u的范围
                u = 1
            psi = psi_d
            Vy = Va * math.sin(psi) + u * dt
            if abs(Vy) >= Va:
                Vy = Va
                Vx = 0
            else:
                Vx = np.sign(math.cos(psi)) * math.sqrt(Va ** 2 - Vy ** 2)
            Px = Px + Vx * dt
            Py = Py + Vy * dt
            pos_record.append([Px, Py])
            aim_record.append([xt, yt])
    # 点采样和绘图
    pos_plot = []
    aim_plot = []
    num = 25
    gap = int(len(pos_record) / num)
    for i in range(num):
        pos_plot.append(pos_record[i * gap])
        aim_plot.append(aim_record[i * gap])
    pos_plot = np.array(pos_plot).T
    aim_plot = np.array(aim_plot).T
    path = np.array(Path).T
    plt.plot(path[0], path[1])
    plt.plot(pos_plot[0], pos_plot[1], '--')
    plt.plot(aim_plot[0], aim_plot[1], '*')
    plt.legend(['目标轨迹', '实际轨迹', '目标航点'])
    plt.axis('equal')
    plt.grid()
    plt.show()


if __name__ == "__main__":
    Path = [[0, 0], [10, 15], [15, 20], [20, 5]]
    move_by_path([6, 3], 3, 0.5, Path, delta=0.1)
    # move_to_point([3, 3], 3, 0, [0, 0], [10, 15])
```

CarrotChasing算法迁移至AirSim仿真环境：

```python
def move_by_path(client, Va, Path, Pz, delta=0.8, K=0.8, K2=0, dt=0.02):
    def distance(A, B):
        return math.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)

    def myatan(A, B):
        x1, y1, x2, y2 = A[0], A[1], B[0], B[1]
        if x1 != x2:
            if x1 > x2:
                return math.atan((y1 - y2) / (x1 - x2)) + math.pi
            else:
                return math.atan((y1 - y2) / (x1 - x2))
        if x1 == x2 and y1 == y2:
            return None
        if x1 == x2 and y1 != y2:
            if y1 > y2:
                return -math.pi / 2
            else:
                return math.pi / 2

    state = client.simGetGroundTruthKinematics()
    psi = airsim.to_eularian_angles(state.orientation)[2]
    Px = state.position.x_val
    Py = state.position.y_val
    Wb = [Px, Py]

    for i in range(len(Path)):
        Wa = Wb
        Wb = [Path[i].x_val, Path[i].y_val]
        theta = myatan(Wa, Wb)
        while True:
            theta_u = myatan(Wa, [Px, Py])
            if theta_u == None:
                theta_u = theta
            beta = theta - theta_u
            Ru = distance(Wa, [Px, Py])
            R = Ru * math.cos(beta)
            e = Ru * math.sin(beta)
            print(Px, Py, e)
            xt = Wa[0] + (R + delta) * math.cos(theta)
            yt = Wa[1] + (R + delta) * math.sin(theta)
            if i == len(Path) - 1:
                if (Px - Wb[0]) * (Wb[0] - Wa[0]) > 0 \
                    or (Py - Wb[1]) * (Wb[1] - Wa[1]) > 0:
                    break
            elif (xt - Wb[0]) * (Wb[0] - Wa[0]) > 0 \
                    or (yt - Wb[1]) * (Wb[1] - Wa[1]) > 0:
                break
            psi_d = myatan([Px, Py], [xt, yt])
            u = K * (psi_d - psi) * Va + K2 * e
            if u > 1:  # 限制u的范围
                u = 1
            psi = psi_d
            Vy = Va * math.sin(psi) + u * dt
            if abs(Vy) >= Va:
                Vy = Va
                Vx = 0
            else:
                Vx = np.sign(math.cos(psi)) * math.sqrt(Va ** 2 - Vy ** 2)
            client.moveByVelocityZAsync(Vx, Vy, Pz, dt).join()
            # 画图
            plot_p1 = [airsim.Vector3r(Px, Py, Pz)]
            state = client.simGetGroundTruthKinematics()
            Px = state.position.x_val
            Py = state.position.y_val
            plot_p2 = [airsim.Vector3r(Px, Py, Pz)]
            client.simPlotArrows(plot_p1, plot_p2, arrow_size=8.0, color_rgba=[0.0, 0.0, 1.0, 1.0])
            client.simPlotLineList(plot_p1 + plot_p2, color_rgba=[1.0, 0.0, 0.0, 1.0], is_persistent=True)

client = airsim.MultirotorClient() 
client.confirmConnection()       
client.reset()
client.enableApiControl(True)     
client.armDisarm(True)           
client.takeoffAsync().join()
client.moveToZAsync(-3, 1).join()
client.simSetTraceLine([1, 0, 0, 1], thickness=5)
client.simFlushPersistentMarkers()

points = [airsim.Vector3r(5, 0, -3),
          airsim.Vector3r(5, 8, -3),
          airsim.Vector3r(8, 12, -3),
          airsim.Vector3r(4, 9, -3)]
client.simPlotPoints(points, color_rgba=[0, 1, 0, 1], size=30, is_persistent=True)

move_by_path(client, 10, points, -3)

client.goHomeAsync().join()      
client.landAsync().join()         
client.armDisarm(False)          
client.enableApiControl(False)
```

参考文献：

[1] Bhadani R . Path Planning of Unmanned System using Carrot-chasing Algorithm[J].  2020.
