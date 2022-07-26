### 1.LQR控制器算法原理推导

#### 1.1 状态反馈控制

连续线性系统的状态空间表示为

$$
\left
\{
\begin{aligned}
\dot{x}&=Ax+Bu
\\y&=Cx+Du
\end{aligned}
\right.
\\
x(t)\in R^n,u(t)\in R^m

$$

对其设计状态反馈控制器

$$
u=w-Kx

$$

其中$w$为设定值，$K$为反馈矩阵，得到的结构图如下所示：

![image-20220630163853280](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220630163853280.png)

设计状态反馈的目标为，通过设计反馈矩阵$K$，使得闭环系统的极点为期望的极点位置。

#### 1.2 连续时间系统LQR

LQR（Linear quadratic regulator，线性二次型调节器）设计的目标为，找到一组控制量$u_0,u_1,···$，使得状态量$x_0,x_1,···$能够快速、稳定地趋近并保存平衡状态，且控制量尽可能小。

其优化目标为

$$
min\quad J=\frac{1}{2}\int_0^\infty (x^TQx+u^TRu)dt

$$

其中，矩阵$Q$为半正定的状态加权矩阵，$R$为正定的控制量加权矩阵，且通常取为对角阵，需根据问题的需求进行设计。接下来对算法进行推导。

使用1.1中的状态反馈，取$w=0$以简化问题，将$u=-Kx$带入优化目标后，得到

$$
J=\frac{1}{2}\int_0^\infty x^T(Q+K^TRK)xdt\qquad (1)

$$

假设存在一个常数矩阵$P$，

$$
\frac{d}{dt}(x^TPx)=-x^T(Q+K^TRK)x \qquad (2)

$$

则

$$
\begin{aligned}
J&=\frac{1}{2}\int_0^\infty -\frac{d}{dt}(x^TPx)dt
\\&=-\frac{1}{2}x^TPx|_0^\infty
\end{aligned}

$$

为使得$J$达到最小值，$t\rightarrow \infty$时$x(t)$应趋于0，故

$$
J=\frac{1}{2}x^T(0)Px(0)\qquad (3)

$$

将(2)式左侧微分展开后得到

$$
\dot{x}^TPx+x^TP\dot{x}+x^TQx+x^TK^TRK^Tx=0\qquad (4)

$$

又因为

$$
\dot{x}=Ax+Bu=(A-BK)x=A'x\qquad (5)

$$

代入(4)式得到s

$$
x^TA'^TPx+x^TPA'x+x^TQx+x^TK^TRK^Tx=0\qquad

$$

整理得到

$$
x^T(A'^TP+PA'+Q+K^TRK^T)x=0\qquad

$$

故

$$
A'^TP+PA'+Q+K^TRK^T=0

$$

即

$$
(A-BK)^TP+P(A-BK)+Q+K^TRK^T=0
\\A^TP+PA+Q+K^TRK-K^TB^TP-PBK=0

$$

取$K=R^{-1}B^TP$时上式取最小值，此时上式化为

$$
A^TP+PA+Q-PBR^{-1}B^TP=0

$$

可求解得到矩阵$P$。

故连续时间系统的LQR算法流程为

1. 根据实际问题选择参数矩阵$Q,R$；
2. 求解方程$A^TP+PA+Q-PBR^{-1}B^TP=0$得到矩阵$P$；
3. 计算反馈矩阵$K=R^{-1}B^TP$；
4. 计算控制量$u=-Kx$

#### 1.3 离散时间系统无限时间LQR

离散时间系统的状态空间描述为

$$
X(k+1)=AX(k)+Bu(k)

$$

LQR计算的优化目标为

$$
min\quad J=\frac{1}{2}\sum_{k=1}^\infty [x(k)^TQx(k)+u(k)^TRu(k)]

$$

这个问题可以用线性规划的方法求解，求解过程比较复杂，也没有深究，感兴趣的话可以看这个视频：https://www.bilibili.com/video/BV1P54y1m7CZ?share_source=copy_web

该问题求解的结果为

$$
P=Q+A^TPA-A^TPB(R+B^TPB)^{-1}B^TPA
\\K=(R+B^TPB)^{-1}B^TPA
\\u(k)=-KX(k)

$$

离散时间无限时间系统的LQR算法（DLQR）流程为

1. 根据实际问题选择参数矩阵$Q,R$；
2. 根据方程$P=Q+A^TPA-A^TPB(R+B^TPB)^{-1}B^TPA$，计算矩阵$P$；
3. 计算反馈矩阵$K=(R+B^TPB)^{-1}B^TPA$；
4. 计算控制量$u=-KX(k)$

### 2.无人机轨迹跟踪控制器DLQR算法设计

取无人机的位置和速度为状态量：$x=[p, v]^T$，加速度为输入量$u=a$，则系统状态方程为

$$
\begin{aligned}
&x(k+1)=Ax(k)+Bu(k)\qquad(1)
\\&x(k)=\begin{bmatrix}p_x(k)\\p_y(k)\\v_x(k)\\v_y(k)\end{bmatrix}
,u(k)=\begin{bmatrix}a_x(k)\\a_y(k)\end{bmatrix}
\\&A=\begin{bmatrix}1&0&dt&0\\0&1&0&dt\\0&0&1&0\\0&0&0&1\end{bmatrix}
,B=\begin{bmatrix}0&0\\0&0\\dt&0\\0&dt\end{bmatrix}
\end{aligned}

$$

假设要跟踪的目标轨迹为$x_{des}=[p_{des}, v_{des}]^T$，加速度为$a_{des}$，则$x_{dex},a_{dex}$满足

$$
x_{des}(k+1)=Ax_{des}(k)+Ba_{des}(k)\qquad(2)

$$

将(1)(2)两个式子相减得到：

$$
[x_{des}(k+1)-x(k+1)]=A[x_{des}(k)-x(k)]+B(a_{des}(k)-u(k))

$$

上式可以看作是一个以$x_{des}-x$为状态变量的状态方程，设计LQR控制器使得$x_{des}-x$趋于0，其优化目标为：

$$
min\quad J=\frac{1}{2}\sum_{k=1}^\infty \{[x_{des}(k)-x(k)]^TQ[x_{des}(k)-x(k)]
+[a_{des}(k)-u(k)]^TR[a_{des}(k)-u(k)]\}

$$

其求解结果为$a_{dex}(k)-u(k)=-K[x_{dex}(k)-x(k)]$，则四旋翼的系统输入量为$u(k)=-K[x(k)-x_{dex}(k)]+a_{dex}(k)$，可根据1.3中的方法计算反馈矩阵和系统输入量。

加权矩阵可取

$$
Q=\begin{bmatrix}1&0&0&0\\0&1&0&0\\0&0&1&0\\0&0&0&1\end{bmatrix},
R=\begin{bmatrix}0.4&0\\0&0.4\end{bmatrix}

$$

### 3.使用加速度进行无人机控制

#### 3.1 四旋翼姿态解算

根据四旋翼的横滚、俯仰、偏航三个角度的计算，可以得到四旋翼的姿态，横滚角$\phi$、俯仰角$\theta$、偏航角$\psi$的示意如下图所示：

![image-20220701113706395](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220701113706395.png)

假设一向量$t$，其在世界坐标系world和机体坐标系body下均为$(t_1,t_2,t_3)^T$。经过偏航、俯仰、横滚，即依次绕$x,y,z$轴旋转后，在世界坐标系下得到向量$t^w$，在机体坐标系下向量$t$依然为$(t_1,t_2,t_3)^T$。需计算机体坐标系到世界坐标系之间的旋转矩阵，使$t^w=R·t$。

首先计算向量$t$绕$x$轴旋转的旋转矩阵$R_x$，将其分解为向量

$$
t_x=(t_1,0,0)^T,t_y=(0,t_2,0)^T,t_z=(0,0,t_3)^T

$$

容易计算得三个分量绕$x$轴旋转角度$\phi$后，转换为向量

$$
t_x^w=(t_1,0,0)^T,t_y^w=(0,t_2·cos\phi,t_2·sin\phi)^T,t_z^w=(0,-t_3·sin\phi,t_3·cos\phi)^T
\\t'=t_x'^w+t_y^w+t_z^w=(t_1,t_2·cos\phi-t_3·sin\phi,t_2·sin\phi+t_3·cos\phi)^T

$$

故

$$
R_x=
\begin{bmatrix}
1&0&0\\
0&cos\phi&-sin\phi\\
0&sin\phi&cos\phi
\end{bmatrix}

$$

同理，可计算得

$$
R_y=
\begin{bmatrix}
cos\theta&0&sin\theta\\
0&1&0\\
-sin\theta&0&cos\theta
\end{bmatrix},
R_z=
\begin{bmatrix}
cos\psi&-sin\psi&0\\
sin\psi&cos\psi&0\\
0&0&1
\end{bmatrix}

$$

则向量$t$依次绕$x,y,z$轴旋转的旋转矩阵为

$$
R=R_zR_yR_x

$$

故机体坐标系下的向量$t$，变换到世界坐标系下向量$t'$，变换矩阵为$R=R_zR_yR_x$。

#### 3.2 根据四旋翼加速度解算姿态角

如果坐标系、欧拉角的定义、坐标轴旋转的方向等不同，会导致旋转矩阵的计算结果不同，故3.1中的结果不能直接搬移到其他坐标系中。AirSim中， 世界坐标系$x,y,z$三个坐标轴的朝向分别为北、东、地，机体坐标系$z,y,z$三个坐标轴的朝向分别为前、右、下，滚转角、俯仰角、偏航角的旋转方向分别为逆时针、顺时针、逆时针（视线沿坐标轴的反方向观察）。按照3.1中的计算方法，计算得三个旋转矩阵为

$$
R_x=
\begin{bmatrix}
1&0&0\\
0&cos\phi&-sin\phi\\
0&sin\phi&cos\phi
\end{bmatrix}，
R_y=
\begin{bmatrix}
cos\theta&0&-sin\theta\\
0&1&0\\
sin\theta&0&cos\theta
\end{bmatrix},
R_z=
\begin{bmatrix}
cos\psi&sin\psi&0\\
-sin\psi&cos\psi&0\\
0&0&1
\end{bmatrix}

$$

这里只考虑无人机水平运动的情况，即无人机保持一定高度，只存在水平方向的加速度，竖直方向加速度为0。

在机体坐标系下，无人机的四个旋翼产生的升力沿$-x$方向，故加速度$a^b=(0,0,-k)^T$，$k$为未知参数。

将向量$a^b$变换到世界坐标系下，

$$
a^w=Ra^b=-k·(m_1,m_2,cos\phi·cos\theta)^T
\\(m_1,m_2)^T=
\begin{bmatrix}
cos\psi&sin\psi\\
-sin\psi&cos\psi
\end{bmatrix}·
\begin{bmatrix}
-sin\theta·cos\phi\\
-sin\phi
\end{bmatrix}

$$

在世界坐标系下，无人机同时受到重力和旋翼的升力，其总加速度为

$$
a=g+a^w=(-k·m_1,-k·m_2,-k·cos\phi·cos\theta+g)^T

$$

由于无人机在$z$轴上加速度为0，故

$$
-k·cos\phi·cos\theta+g=0
\\k=\frac{g}{cos\phi·cos\theta}

$$

无人机在其他两个方向的加速度为

$$
\begin{aligned}
(a_x,a_y)^T&=
-k·(m_1,m_2)^T
\\&=-k·\begin{bmatrix}
cos\psi&sin\psi\\
-sin\psi&cos\psi
\end{bmatrix}·
\begin{bmatrix}
-sin\theta·cos\phi\\
-sin\phi
\end{bmatrix}
\\&=g·\begin{bmatrix}
cos\psi&sin\psi\\
-sin\psi&cos\psi
\end{bmatrix}·
\begin{bmatrix}
tan\theta\\
tan\phi/cos\theta
\end{bmatrix}
\end{aligned}

$$

然后可以根据加速度解算出无人机的姿态角。

#### 3.3 AirSim中四旋翼的姿态角控制

解算出四旋翼的姿态角后，可使用如下的函数控制；

```python
moveByRollPitchYawZAsync(roll, pitch, yaw, z, duration, vehicle_name)
```

四个参数分别为滚转角、俯仰角、偏航角、高度、持续时间。

综上，使用LQR实现无人机轨迹跟踪的算法流程为：

![image-20220705124955050](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220705124955050.png)

参考来源：

四旋翼轨迹跟踪：https://zhuanlan.zhihu.com/p/394491146

连续时间LQR控制算法原理推导：https://blog.csdn.net/weixin_42301220/article/details/124542242

离散时间无限时间系统LQR控制算法推导：https://www.bilibili.com/video/BV1P54y1m7CZ?share_source=copy_web
