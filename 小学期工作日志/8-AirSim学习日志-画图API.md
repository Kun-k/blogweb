之前写轨迹跟踪代码的时候用到了AirSim的绘图API，但是实际上没弄清楚几个API的功能，代码是能跑就算成功。今天专门学习整理一下绘图相关的API。

在编辑器中输入`client.simplot`后，检索到相关的函数如下：

![image-20220708101139346](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220708101139346.png)

#### 画线`simPlotLineList`

```python
def simPlotLineList(self, points, color_rgba=[1.0, 0.0, 0.0, 1.0], thickness = 5.0, duration = -1.0, is_persistent = False)
```

`client.py`文件中对该函数的解释是：

```python
Plots a line strip in World NED frame, defined from points[0] to points[1], points[2] to points[3], ... , points[n-2] to points[n-1]
```

也就是在全局坐标系(NED,北东地)下，按$P0-P1,P2-P3,···$的顺序画线。

* 参数`points`的类型为`list[Vector3r]`，为要绘制的点，由包含三维坐标信息的`airsim.Vector3r`组成的列表，且必须为偶数个点。
* 参数`color_rgba`的类型为`list`，分别为RGB通道和不透明度，值范围为0-1。
* 参数`thickness`的类型为`float`，表示线条粗细。
* 参数`duration`的类型为`float`，表示绘制的持续时间。
* 参数`is_persistent`的类型为`bool`，如果为真，则将在无限时间内绘制。

前三个参数比较好理解，但是后两个参数的解释就不太清楚了。我自己去仿真环境里跑一下才知道，这两个参数说的是绘图在场景中存留的时间，如果`is_persistent`为真，则绘图会一直存在（除非手动设置擦除绘图）；如果`is_persistent`为假，则要看参数`duration`，它的值表示绘图在场景中存留的时间长短。

![image-20220708104351938](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220708104351938.png)

<center>
（simPlotLineList绘制，四个点画出两条线）
</center>

#### 画线`simPlotLineStrip`

```python
def simPlotLineStrip(self, points, color_rgba=[1.0, 0.0, 0.0, 1.0], thickness = 5.0, duration = -1.0, is_persistent = False)
```

`client.py`文件中对该函数的解释是：

```python
Plots a line strip in World NED frame, defined from points[0] to points[1], points[1] to points[2], ... , points[n-2] to points[n-1]
```

与上面的`simPlotLineList`只有一个不同，它的画线是按$P0-P1,P1-P2,···$的顺序，是连续的。函数参数的含义也都与`simPlotLineList`一致。

![image-20220708104327101](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220708104327101.png)

<center>
（simPlotLineStrip绘制，四个点画出三条线）
</center>

#### 画箭头`simPlotArrows`

```python
def simPlotArrows(self, points_start, points_end, color_rgba=[1.0, 0.0, 0.0, 1.0], thickness = 5.0, arrow_size = 2.0, duration = -1.0, is_persistent = False)
```

`client.py`文件中对该函数的解释是：

```python
Plots a list of arrows in World NED frame, defined from points_start[0] to points_end[0], points_start[1] to points_end[1], ... , points_start[n-1] to points_end[n-1]
```

给出两组三维点`points_start`和`points_end`，绘制由`points_start`指向`points_end`的箭头。点集的定义与上面两个函数一致，其他参数的含义也一致。

![image-20220708104327101](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220708105315630.png)

<center>
（simPlotArrows绘制，但这箭头很小还很抽象，放大到跟前才看出来）
</center>

#### 画点`simPlotPoints`

```python
def simPlotPoints(self, points, color_rgba=[1.0, 0.0, 0.0, 1.0], size = 10.0, duration = -1.0, is_persistent = False)
```

`client.py`文件中对该函数的解释是：

```python
Plot a list of 3D points in World NED frame
```

意思就是画点。除了参数`size`外，其他参数都与之前一致，`size`表示点的尺寸。

![image-20220708105736320](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220708105736320.png)

<center>
（simPlotPoints绘制，这点是方的，而且无论从哪个角度看都是正方形，没有立体感）
</center>

#### 画字符`simPlotStrings`

```python
def simPlotStrings(self, strings, positions, scale = 5, color_rgba=[1.0, 0.0, 0.0, 1.0], duration = -1.0)
```

`client.py`文件中对该函数的解释是：

```python
Plots a list of strings at desired positions in World NED frame.
```

意思是画一系列string。

* 参数`strings`的类型为`list[String]`，为要绘制的字符串。
* 参数`positions`的类型为`list[Vector3r]`，字符串的位置，与上一个参数一一对应。
* 参数`scale`的类型为`list[float]`，对应字符的大小。

其他两个参数与之前一致。这里没有`is_persistent`这个参数，也就不能持续绘图。但是我发现如果设置`duration`为-1，绘图会持续存在，设置其他负数没有这个效果，前面几个函数也没有这个效果。但问题是画上去之后擦不掉了，用清空绘图的函数也不行...代码还是不能乱写。

![image-20220708110940203](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220708110940203.png)

<center>
（simPlotStrings绘制，同时绘制了对应位置的点）
</center>

#### 绘制变换

有`simPlotTransforms`和`simPlotTransformsWithNames`两个函数，官方文档和`client.py`中都说是`Plots a list of transforms`，但是这个`transforms`到底是指什么也不太清楚，网上也查不到相关的资料。如果以后会使用到再来看这两个函数吧。

#### 清除所有绘图`simFlushPersistentMarkers`

```python
 client.simFlushPersistentMarkers()
```

顾名思义，清除世界中的所有绘图。

