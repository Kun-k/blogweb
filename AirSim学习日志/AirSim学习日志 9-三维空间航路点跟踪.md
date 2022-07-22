之前已经使用Carrot-Chasing方法实现了二维平面内的航路点跟踪，但是为实现其控制，需利用二维平面内的几何关系，计算大量夹角，如果将该控制器推广到三维平面内，计算推导会更加复杂，而且直线与轴或平面的夹角也需要重新考虑。今天学习了一种新的控制方法，可以避免复杂的计算，也更加易于理解。

这一方法的参考来源为《多旋翼飞行器设计与控制》 P284 路径规划中的直线路径跟踪部分，其基本思想与Carrot-Chasing一样，将两个航路点连接成线，要求无人机跟踪这条直线。

### 1.控制原理

首先将多旋翼进行如下的建模：
$$
\left\{
\begin{aligned}
\dot{p}=v
\\\dot{v}=u
\end{aligned}
\right.
$$
$p,v,u\in \R^3$分别表示无人机的位置、速度和加速度。假设其要跟随的路径为$P_{wp,last}-P_{wp}$，如下图所示，图片是书中讲解该部分的插图：

![image-20220711163348591](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220711163348591.png)

计算垂足$P_{wp,prep}$为
$$
P_{wp,prep}=P_{wp}+(P_{wp,last}-P_{wp})\frac{(P-P_{wp})^T(P_{wp,last}-P_{wp})}{||P_{wp-P_{wp,last}}||^2}
$$
则
$$
\begin{align}
P-P_{wp,prep}&=(P-P_{wp})+(P_{wp,last}-P_{wp})\frac{(P-P_{wp})^T(P_{wp,last}-P_{wp})}{||P_{wp-P_{wp,last}}||^2}
\\&=A(P-P_{wp})
\\A&=I_3-
\frac{(P_{wp,last}-P_{wp})(P_{wp,last}-P_{wp})^T}{||P_{wp-P_{wp,last}}||^2}
\end{align}
$$
控制器输出为
$$
u=-\frac{1}{k_2}sat_{gd}(k_0\hat P_{wp}+k_1A\hat{P}_{wp},a0)-\frac{1}{k_2}v
$$
其中，$\hat P_{wp}=P-P_{wp}$，$sat_{gt}(m,a)$函数的作用为限制控制器输出幅度，其表示形式为
$$
sat_{gt}(m,a)=
\left\{
\begin{aligned}
m,\quad&if\quad||m||_\infty \leq a
\\a\frac{m}{||m||_\infty},\quad&if \quad||m||_\infty>a
\end{aligned}
\right.
$$
该控制器的稳定性可以通过李雅普诺夫稳定性证明，书中有详细证明。

这一控制器同时考虑到了三个因素：

一是无人机当前的速度，其符号与控制器输出的符号相反，则无人机当前速度与目标速度的方向差距越大，控制器的响应就越大；

二是无人机与目标航点的距离，即$\hat{P}_{wp}$，对应控制器参数$k_0$，该参数越大，无人机飞向目标航点的趋势就越大。

三是无人机与目标轨迹的距离，即$A\hat{P}_{wp}=P-P_{wp,prep}$，对应控制器参数$k_1$，该参数越大，无人机飞向目标轨迹的趋势就越大。

### 2.实现效果

设置参数$k_0=0,k_1=1,k_2=0.8$，初始状态为$P=[1,1,1],v=[1,1,1]$，跟踪直线为$[0,0,0]-[10,15,15]$，得到的效果如下，无人机直接飞向目标轨迹，飞向目标航点的趋势很小。

![image-20220711170045481](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220711170045481.png)

设置参数$k_0=1,k_1=0,k_2=0.8$，得到的效果如下，无人机飞向目标轨迹的趋势很小，直接飞向目标航点，在最后才与目标轨迹产生相交。

![image-20220711170218489](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220711170218489.png)

设置参数$k_0=1,k_1=2,k_2=0.8$，得到的效果如下。

![image-20220711170619262](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220711170619262.png)

为实现跟踪多个航路点组成的路径，增加参数$\delta$，当$P_{wp,prep}$与$P_{wp}$距离小于$\delta$时，开始对下一个航路点进行跟踪。设置参数$k_0=1,k_1=2,k_2=0.8,\delta=0.5$，，得到的效果如下。

![image-20220711170910146](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220711170910146.png)

将代码修改后迁移到AirSim中，跟踪航路点为

```python
points = [airsim.Vector3r(5, 0, -3),
          airsim.Vector3r(3, 10, -1),
          airsim.Vector3r(8, 12, -7),
          airsim.Vector3r(-5, 9, -2)]
```

设置参数为

```python
K0=1.5, K1=4, K2=0.6, a0=1, delta=0.7
```

`a0`为控制器限幅参数，这里与二维的控制器一个很重要的不同点在于，二维要求固定飞行器速率，而这里无法设置飞行器速率，速度由控制器计算得到，我们只能进行控制输出的限幅。设置控制器限幅的范围越大，飞行器速度可能越大。

这里的跟踪效果如下：

![image-20220711171345648](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220711171345648.png)

还是实现了比较好的航路点跟踪，但是在$z$轴上的误差比较大。这与四旋翼控制方法是有关的，由于控制器输出为加速度，而AirSim中没有相关的接口，这里依然使用了DLQR方法中用过的加速度控制方法，需先通过加速度解算出位姿，再通过AirSim中的姿态角控制接口进行控制。但是AirSim中提供的相关接口不能实现$z$轴方向的加速度控制，在$z$轴上只能进行位置控制或高度油门控制。这里采用了简单的高度位置控制，认为目标高度为

```python
z_cmd = P[2] + (V[2] + U_cmd[2] * dt) * dt
```

然后多次尝试、设置合理的控制器参数$k_2$，使四旋翼轨迹与目标轨迹的误差尽可能小。

### 3.代码

python代码实现：

```python
def move_by_path_3d(P, V, Path, K0=2, K1=2, K2=1, dt=0.1, a0=2, delta=0.5):
    def distance(A, B):
        return math.sqrt((A[0] - B[0]) ** 2 +
                         (A[1] - B[1]) ** 2 +
                         (A[2] - B[2]) ** 2)

    P = np.matrix(P).T
    pos_record = [P]
    e_record = []
    V = np.matrix(V).T
    I3 = np.matrix([[1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])

    for i in range(len(Path) - 1):
        Wa = np.matrix(Path[i]).T
        Wb = np.matrix(Path[i + 1]).T
        A = I3 - (Wa - Wb).dot((Wa - Wb).T) / (distance(Wa, Wb) ** 2)
        Pt = P - Wb
        e = np.linalg.norm(A.dot(Pt))
        e_record.append(e)
        d = np.linalg.norm(Pt - A.dot(Pt))
        print('i,', 'Start:', Path[i], ',Aim:', Path[i + 1])
        print('\tP:', P, 'V:', V, 'e:', e)
        while d >= delta:
            Pt = P - Wb
            U1 = K0 * Pt + K1 * A.dot(Pt)
            if np.linalg.norm(U1, ord=np.inf) > a0:
                U1 = U1 * a0 / np.linalg.norm(U1, ord=np.inf)
            U = -(U1 + V) / K2
            V = V + U * dt
            P = P + V * dt
            e = np.linalg.norm(A.dot(Pt))
            e_record.append(e)
            d = np.linalg.norm(Pt - A.dot(Pt))
            print('\tP:', P, 'V:', V, 'e:', e)
            pos_record.append(P)
    pos_record = np.array(pos_record).T[0]
    path_plot = np.array(Path).T
    ax = plt.subplot(projection='3d')
    ax.plot(path_plot[0], path_plot[1], path_plot[2])
    ax.plot(pos_record[0], pos_record[1], pos_record[2], '--')
    plt.show()
    plt.plot(e_record)
    plt.show()

if __name__ == "__main__":
    Path = [[0, 0, 0], [10, 15, 15], [15, 20, 20], [20, 5, 5]]
    move_by_path_3d([1, 1, 1], [1, 1, 1], Path, K0=1, K1=2, K2=0.8)
```

迁移至AirSim中：

```python
def get_state(client):
    # 获取无人机状态
    DIG = 6
    State = client.getMultirotorState()
    kinematics = State.kinematics_estimated
    state = {
        "timestamp": str(State.timestamp),
        "position": [round(ele, DIG) for i, ele in
                     enumerate(kinematics.position.to_numpy_array().tolist())],
        "orientation": [round(i, DIG) for i in airsim.to_eularian_angles(kinematics.orientation)],
        "linear_velocity": [round(i, DIG) for i in kinematics.linear_velocity.to_numpy_array().tolist()],
        "linear_acceleration": [round(i, DIG) for i in kinematics.linear_acceleration.to_numpy_array().tolist()],
        "angular_velocity": [round(i, DIG) for i in kinematics.angular_velocity.to_numpy_array().tolist()],
        "angular_acceleration": [round(i, DIG) for i in kinematics.angular_acceleration.to_numpy_array().tolist()]
    }
    return state


def move_by_acceleration_horizontal(client, ax_cmd, ay_cmd, az_cmd, z_cmd, duration=1):
    # 读取自身yaw角度
    state = get_state(client)
    angles = state['orientation']
    yaw_my = angles[2]
    g = 9.8  # 重力加速度
    sin_yaw = math.sin(yaw_my)
    cos_yaw = math.cos(yaw_my)
    A_psi = np.array([[cos_yaw, sin_yaw], [-sin_yaw, cos_yaw]])
    A_psi_inverse = np.linalg.inv(A_psi)
    angle_h_cmd = 1 / (g - az_cmd) * np.dot(A_psi_inverse, np.array([[ax_cmd], [ay_cmd]]))
    theta = math.atan(angle_h_cmd[0, 0])
    phi = math.atan(angle_h_cmd[1, 0] * math.cos(theta))
    # client.moveToZAsync(z_cmd, vz).join()
    client.moveByRollPitchYawZAsync(phi, theta, 0, z_cmd, duration).join()


def move_by_path_3d(client, Path, K0=1.5, K1=4, K2=0.6, dt=0.5, a0=1, delta=0.7):
    def distance(A, B):
        return math.sqrt((A[0] - B[0]) ** 2 +
                         (A[1] - B[1]) ** 2 +
                         (A[2] - B[2]) ** 2)
    state = get_state(client)
    P = state['position']
    V = state['linear_velocity']
    Wb = P
    Wb_m = np.matrix(Wb).T
    P_m = np.matrix(P).T
    V_m = np.matrix(V).T
    I3 = np.matrix([[1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]])
    for i in range(len(Path)):
        Wa = Wb
        Wb = [Path[i].x_val, Path[i].y_val, Path[i].z_val]
        Wa_m = Wb_m
        Wb_m = np.matrix(Wb).T
        A = I3 - (Wa_m - Wb_m).dot((Wa_m - Wb_m).T) / (distance(Wa_m, Wb_m) ** 2)
        Pt = P_m - Wb_m
        e = np.linalg.norm(A.dot(Pt))
        d = np.linalg.norm(Pt - A.dot(Pt))
        print('i,', i, 'Start:', Wa, ',Aim:', Wb)
        print('\tP:', P, 'V:', V, 'e:', e)
        while d >= delta or \
                (i == len(Path) - 1
                 and ((P[0] - Wb[0]) * (Wb[0] - Wa[0]) < 0
                      or (P[1] - Wb[1]) * (Wb[1] - Wa[1]) < 0
                      or (P[2] - Wb[2]) * (Wb[2] - Wa[2]) < 0)):
            Pt = P_m - Wb_m
            U1 = K0 * Pt + K1 * A.dot(Pt)
            if np.linalg.norm(U1, ord=np.inf) > a0:
                U1 = U1 * a0 / np.linalg.norm(U1, ord=np.inf)
            U = -(U1 + V_m) / K2
            U_cmd = np.array(U)[:, 0]
            z_cmd = P[2] + (V[2] + U_cmd[2] * dt) * dt
            move_by_acceleration_horizontal(client, U_cmd[0], U_cmd[1], U_cmd[2], z_cmd, dt)
            e = np.linalg.norm(A.dot(Pt))
            d = np.linalg.norm(Pt - A.dot(Pt))
            print('\tP:', P, 'V:', V, 'e:', e, 'U:', U_cmd)
            # 画图
            plot_p1 = [airsim.Vector3r(P[0], P[1], P[2])]
            state = get_state(client)
            P = state['position']
            V = state['linear_velocity']
            P_m = np.matrix(P).T
            V_m = np.matrix(V).T
            plot_p2 = [airsim.Vector3r(P[0], P[1], P[2])]
            client.simPlotArrows(plot_p1, plot_p2, arrow_size=8.0, color_rgba=[0.0, 0.0, 1.0, 1.0])
            client.simPlotLineStrip(plot_p1 + plot_p2, color_rgba=[1.0, 0.0, 0.0, 1.0], is_persistent=True)

client = airsim.MultirotorClient()  # 创建连接
client.confirmConnection()          # 检查连接
client.reset()
client.enableApiControl(True)       # 获取控制权
client.armDisarm(True)              # 电机启转
client.takeoffAsync().join()        # 起飞
client.moveToZAsync(-3, 1).join()   # 上升到3米高度
client.simSetTraceLine([1, 0, 0, 1], thickness=5)
client.simFlushPersistentMarkers()  # 清空画图

# 三维航路点跟踪
points = [airsim.Vector3r(5, 0, -3),
          airsim.Vector3r(3, 10, -1),
          airsim.Vector3r(8, 12, -7),
          airsim.Vector3r(-5, 9, -2)]
client.simPlotPoints(points, color_rgba=[0, 1, 0, 1], size=30, is_persistent=True)
client.simPlotLineStrip(points, color_rgba=[0, 1, 0, 1], thickness=5, is_persistent=True)
move_by_path_3d(client, points)
```

