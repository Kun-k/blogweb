### 1.无人机真值状态读取

真值状态即无误差的无人机状态，通过以下的函数可读取无人机或无人车的真值状态，

```python
state = client.simGetGroundTruthKinematics()
```

实际运行一次可以得到如下的结果：

```
<KinematicsState> {   'angular_acceleration': <Vector3r> {   'x_val': 0.0,
    'y_val': 0.0,
    'z_val': 0.0},
    'angular_velocity': <Vector3r> {   'x_val': 0.0,
    'y_val': 0.0,
    'z_val': 0.0},
    'linear_acceleration': <Vector3r> {   'x_val': 0.0,
    'y_val': 0.0,
    'z_val': 0.41852283477783203},
    'linear_velocity': <Vector3r> {   'x_val': 0.0,
    'y_val': 0.0,
    'z_val': 0.01781749166548252},
    'orientation': <Quaternionr> {   'w_val': 1.0,
    'x_val': 0.0,
    'y_val': 0.0,
    'z_val': 0.0},
    'position': <Vector3r> {   'x_val': 0.0,
    'y_val': 0.0,
    'z_val': -1.6848435401916504}}
```

可以看出其中包含六个属性：

```
angular_acceleration		# 角加速度
	x_val:	滚转角加速度
	y_val:	俯仰角加速度
	z_val:	偏航角加速度
angular_velocity    		# 角速度
	x_val:	滚转角速度
	y_val:	俯仰角速度
	z_val:	偏航角速度
linear_acceleration 		# 加速度
	x_val:	x轴方向加速度（正北）
	y_val:	y轴方向加速度（正东）
	z_val:	z轴方向加速度（地面）
linear_velocity     		# 速度
	x_val:	x轴方向速度（正北）
	y_val:	y轴方向速度（正东）
	z_val:	z轴方向速度（地面）
position            		# 位置
	x_val:	x轴方向位置（正北）
	y_val:	y轴方向位置（正东）
	z_val:	z轴方向位置（地面）
```

姿态角`orientation`读出的结果为四元数，AirSim中给出了将四元数转换为欧拉角的函数

```python
(pitch, roll, yaw) = airsim.to_eularian_angles(state.orientation)
```

这里需要注意位姿角、角速度和角加速度的方向，在仿真环境中测试得到，数据为正时转向的对应关系为：

```
位姿角控制输入值：滚转角：逆时针；俯仰角：顺时针；偏航角：逆时针
角加速度读取值：滚转角：顺时针；俯仰角：顺时针；偏航角：顺时针
角速度读取值：滚转角：逆时针；俯仰角：逆时针；偏航角：顺时针
角度读取值：滚转角：逆时针；俯仰角：逆时针；偏航角：顺时针
```

观察角度方向时，视线沿坐标轴的反方向。同一个仿真环境下的四种角度方向不一致，真是挺奇怪的，也或许我在读取的过程中有误读或理解错误的地方。

### 2.无人机状态估计值读取

在实际情况下，我们无法直接获取无人机无误差的状态，只能通过传感器估计无人机状态。AirSim中可通过以下的函数读取：

```python
 state = client.getMultirotorState(vehicle_name='')
```

得到的结果为：

```
<MultirotorState> {   'collision': <CollisionInfo> {   'has_collided': False,
    'impact_point': <Vector3r> {   'x_val': 0.0,
    'y_val': 0.0,
    'z_val': 0.0},
    'normal': <Vector3r> {   'x_val': 0.0,
    'y_val': 0.0,
    'z_val': 0.0},
    'object_id': -1,
    'object_name': '',
    'penetration_depth': 0.0,
    'position': <Vector3r> {   'x_val': 0.0,
    'y_val': 0.0,
    'z_val': 0.0},
    'time_stamp': 0},
    'gps_location': <GeoPoint> {   'altitude': 122.14649963378906,
    'latitude': 47.6414680362309,
    'longitude': -122.14016508957918},
    'kinematics_estimated': <KinematicsState> {   'angular_acceleration': <Vector3r> {   'x_val': 0.0,
    'y_val': 0.0,
    'z_val': 0.0},
    'angular_velocity': <Vector3r> {   'x_val': 0.0,
    'y_val': 0.0,
    'z_val': 0.0},
    'linear_acceleration': <Vector3r> {   'x_val': 0.0,
    'y_val': 0.0,
    'z_val': 0.0},
    'linear_velocity': <Vector3r> {   'x_val': 0.0,
    'y_val': 0.0,
    'z_val': 0.0},
    'orientation': <Quaternionr> {   'w_val': 1.0,
    'x_val': 0.0,
    'y_val': 0.0,
    'z_val': 0.0},
    'position': <Vector3r> {   'x_val': 0.0,
    'y_val': 0.0,
    'z_val': -0.14649587869644165}},
    'landed_state': 1,
    'rc_data': <RCData> {   'is_initialized': True,
    'is_valid': True,
    'left_z': 0.0,
    'pitch': -0.0,
    'right_z': 0.0,
    'roll': 0.0,
    'switches': 0,
    'throttle': 0.5,
    'timestamp': 0,
    'vendor_id': 'VID_001F',
    'yaw': 0.0},
    'timestamp': 1657094238512604416}
```

其中包含六大类信息信息：

```
collision  #碰撞信息 
    has_collided
    impact_point
    normal
    object_id
    object_name
    penetration_depth
    position
    time_stamp
gps_location  # GPS信息
    altitude
    latitude
    longitude
kinematics_estimated  # 运动信息，包含的运动信息与真实值读取得到的参数含义一致
    angular_acceleration
    angular_velocity
    linear_acceleration
    linear_velocity
    orientation
    position
landed_state  # 着陆信息，0表示在地面，1表示在空中
rc_data  # 遥控器信息
    is_initialized
    is_valid
    left_z
    pitch
    right_z
    roll
    switches
    throttle   
    timestamp
    vendor_id
    yaw
timestamp  # 时间戳
```