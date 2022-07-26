AirSim提供很多API接口，本文将使用python，通过这些接口实现对单个无人机简单飞行的控制。

### 1.python库的安装

需安装两个airsim相关的库：

```
pip install msgpack-rpc-python  
pip install airsim
```

### 2.在Unreal中运行无人机

#### 2.1 仿真运行

在`Documents\AirSim`目录下创建`setting.json`文件，并写入如下的内容：

```
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor"
}
```

`"SimMode": "Multirotor"`表示仿真模式为无人机，如果修改为` "SimMode": "Car"`则为汽车。

之后进入环境目录下，运行VS项目工程文件，并使用`Debug Game Editor`模式开始调试。启动Unreal Editor后，可点击运行按钮运行仿真。在仿真运行时不会显示鼠标，通过“shift+F1”可弹出。

#### 2.2 初始位置设置

进入Unreal Editor后，可对无人机的初始位置进行设置。在界面右侧的“世界大纲视图”中，搜素“PlayStart”，可以看到环境中已经设定的初始位置，如存在多个，保留一个即可。在其下方的“细节”一栏中，可修改参数以调整初始位置，或在关卡中拖动以修改初始位置。

![image-20220629114939841](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220629120155118.png)

#### 2.3 视角设置

Unreal的环境中可以切换五种不同的视角：

* `F`键切换至`FirstPersonView`，第一人称视角；
* `B`键切换至`FlyWithMe`，跟随视角；
* `\`键切换至`GroundObserver`，地面观察者；
* `/`键切换至`SpringArmChase`，弹性机械臂跟随；
* `M`键切换至`Manual`，手动控制，可以通过键盘按键控制观察者的位置和视角：
  * 方向键：前后左右的水平移动；
  * `Page Up/Page Down`：上下的垂直移动；
  * `W,S`：俯仰转动；
  * `A,D`：偏航转动。

### 3.无人机飞行控制

使用python对无人机飞行控制的基本代码逻辑可以分解为以下的步骤，接下来将分别说明如何通过代码实现。

![image-20220629114939841](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220629114939841.png)

#### 3.1 获取与无人机的连接

在第2步中已经运行无人机的基础上，通过`airsim.MultirotorClient()`请求与无人机的连接，建立连接后返回句柄`client`，并通过`client.confirmConnection()`检查连接情况并打印。完整代码如下所示：

```python
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()
```

如果是对汽车进行仿真，则使用`client = airsim.CarClient()`。

#### 3.2 控制权的获取和交还

为确保安全，API控制默认不开启，故使用代码进行控制前需先获取控制权，在控制结束后交回控制权。使用`client.enableApiControl()`设置控制权，如输入参数为`True`则获取控制权，`False`则失去控制权。

```python
client.enableApiControl(True)    # get control
client.enableApiControl(False)   # release control
```

#### 3.3 无人机解锁和锁定

使用`client.armDisarm()`对无人机解锁和锁定，锁定后无人机旋翼停止旋转。

```python
client.armDisarm(True)    # 解锁
client.armDisarm(False)   # 锁定
```

#### 3.4 无人机起飞和降落

代码指令如下所示，其中`.join()`的作用是让该任务完全执行，再执行之后的任务，否则会被后面的任务打断。

```python
client.takeoffAsync().join()   # 起飞
client.landAsync().join()      # 降落
```

#### 3.5 无人机飞行

##### 3.5.1 坐标系

无人机飞行的控制方式可以分为位置控制和速度控制两种，为实现对无人机的控制，首先需要弄清楚坐标系。

这里需要注意三个坐标系：

1. Unreal中的世界坐标系，即Unreal中定义的机体外部环境的坐标系，这一坐标系在Unreal Editor中被用到。其$x,y,z$三个坐标轴的指向分别为北、东、上的方向，单位长度为厘米。
2. AriSim中的全局坐标系，即AirSim中定义的机体外部环境的坐标系，在编写`setting.json`文件和airsim代码时，参考的坐标系为该坐标系。其$x,y,z$三个坐标轴的指向分别北、东、下的方向，单位长度为米。
3. AriSim中的机体坐标系，即以机身朝向定义的坐标系，$x,y,z$三个坐标轴的指向分别为机体前、右、下的方向，单位长度为米。

在编写`setting.json`文件和airsim代码时，应依照AirSim中的全局坐标系，且其$(0,0)$的位置为Unreal Editor中选择的PlayerStart的位置。在仿真运行时，AirSim会将坐标自动转换至Unreal中的世界坐标系。

##### 3.5.2 位置控制

位置控制主要使用到如下的两个函数，同样可以通过`.join()`进行阻塞。接下来对位置控制的三个控制通道进行说明。

```python
client.moveToZAsync()
client.moveToPositionAsync()
```

1. 高度通道

   `moveToZAsync()`用于设置无人机的高度，其可设定$z$坐标和速度两个参数，如

   ```python
   client.moveToZAsync(-3, 2).join()
   ```

   表示设定高度为3，速度为2，由于世界坐标系$z$坐标的朝向为地，故参数为-3。

2. 水平通道

   `moveToPositionAsync()`用于设定无人机的水平位置，对其水平通道进行控制的用法实例为

   ```python
   client.moveToPositionAsync(5, 0, -3, 1).join()
   ```

   表示飞往世界坐标系中$(5, 0, -3)$的位置，即水平位置为$(5,0)$，高度为3，最后一个参数表示速度。

3. 偏航通道

   `moveToPositionAsync()`还可以对偏航通道进行控制，用法示例为：

   ```python
   drivetrain = airsim.DrivetrainType.MaxDegreeOfFreedom
   yaw_mode = airsim.YawMode(True, 10)
   client.moveToPositionAsync(0, 0, -3, 1, drivetrain=drivetrain, yaw_mode=yaw_mode).join()
   ```

   这里用到了`moveToPositionAsync()`的另外两个参数：

   参数`drivetrain`有两种模式可以设置：

   * `airsim.DrivetrainType.ForwardOnly`，表示始终朝向速度方向；

   * `airsim.DrivetrainType.MaxDegreeOfFreedom`，表示手动设置偏航方向。

   参数`yaw_mode`为偏航的模式和参数值，有两种模式可以设置：

   * `airsim.YawMode(True, 10)`，第一个参数设置为`True`表示设置偏航角速度，第二个参数为角速度的值；
   * `airsim.YawMode(False, 10)`，第一个参数设置为`False`表示设置偏航速度，第二个参数为角度的值；

   以上两个参数可以产生四种组合：

   * `airsim.DrivetrainType.ForwardOnly`，`airsim.YawMode`置为`True`，由于第一个参数要求始终朝向速度方向，第二个参数要求无人机以一定角速度持续旋转，二者矛盾，这种情况下无人机不会执行命令。
   * `airsim.DrivetrainType.ForwardOnly`，`airsim.YawMode`置为`False`，此时以无人机的速度方向为参考方向，产生角度为`airsim.YawMode`第二个参数的偏航角。
   * `airsim.DrivetrainType.MaxDegreeOfFreedom`，`airsim.YawMode`置为`True`，第一个参数要求手动设置偏航角度，第二个参数要求设置偏航加速度，故无人机以一定的角速度旋转。
   * `airsim.DrivetrainType.MaxDegreeOfFreedom`，`airsim.YawMode`置为`False`，此时以正北方向为参考方向，产生角度为`airsim.YawMode`第二个参数的偏航角。

##### 3.5.3 速度控制

速度控制可通过函数`moveByVelocityZAsync()`或`moveByVelocityAsync()`实现。

`moveByVelocityZAsync()`函数的前2个参数为无人机在$x,y$方向的速度$vx,vy$，第三个参数为无人机在$z$方向的位置，第四个参数为飞行时间，如以下代码可实现无人机以1m/s速度、在高度3m处沿$x$方向飞3秒钟。

```python
client.moveByVelocityZAsync(1, 0, -3, 3).join()
```

`moveByVelocityZAsync()`同样可以设置偏航，方法与位置控制中设置偏航一致。

`moveByVelocityAsync()`只有第三个参数与`moveByVelocityZAsync()`不同，其第三个参数为无人机在$z$方向的速度。

但是由于无人机达到指令所要求的速度需要一定的调整时间，而其安装该指令飞行的总时长是固定的，故使用速度控制时总会存在一定的误差，而且速度的当前值与设定值差距越大，其误差越大。

##### 3.5.4 悬停

使用`hoverAsync()`函数配合延迟函数可实现悬停，如以下代码可实现悬停3s。

```python
client.hoverAsync().join()
time.sleep(3)
```

