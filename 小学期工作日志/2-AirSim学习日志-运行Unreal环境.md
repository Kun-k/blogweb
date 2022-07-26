完成AirSim的配置后，我们可以在一个已有的Unreal环境中进行仿真。本文将说明如何在一个已有的Unreal环境下进行运行调试。

### 1.获取一个Unreal环境

如果你还没有一个Unreal环境，可以去Unreal的官方商店下载一个免费的环境。本文使用的是通过Unreal官方商店下载的环境。

![image-20220629111717114](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220629111717114.png)



### 2.添加AirSim插件

回到之前编译过的AirSim源码文件夹下，找到文件夹`AirSim\Unreal\Plugins`，将“Plugins”文件夹拷贝到当前的Unreal环境中即可。

![image-20220629112636362](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220629112636362.png)

使用文本编辑器打开“xxx.uproject”文件，修改为如下的内容。其中"EngineAssociation"版本号根据自己项目的情况修改，"EpicSampleNameHash"一项不用管。

```
{
	"FileVersion": 3,
	"EngineAssociation": "4.27",
	"Category": "Samples",
	"Description": "",
	"Modules": [
		{
			"Name": "mountain",
			"Type": "Runtime",
			"LoadingPhase": "Default",
			"AdditionalDependencies": [
                         			"AirSim"
                     		]
		}
	],
	"TargetPlatforms": [
		"MacNoEditor",
		"WindowsNoEditor"
	],
	"Plugins": [
             	{
                     "Name": "AirSim",
                     "Enabled": true
             	}
         	],
	"EpicSampleNameHash": "1226740271"
}
```



### 3.创建VS工程

可以先判断一下你的环境中有没有C++类文件和Visual Studio工程文件：C++源码保存在Source文件夹下，如果没有找到“Source\环境名称\MyClass.cpp”，说明没有C++类文件；如果没有找到“.sln”文件，说明没有VS的工程文件。

双击“xxx.uproject”启动Unreal编辑器，如果还没有C++类文件，则点击“文件-新建C++类”，选择父类为“无”，后续的配置都依照默认即可。

![image-20220629113033470](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220629113033470.png)

接下来生成VS工程文件。将Unreal编辑器和上一步打开的VS 2022全部关闭，右击“xxx.uproject”，在菜单中选择“Generte Visual Studio project files”，即可生成VS工程文件“xxx.sln”。之后的配置和“1-环境配置”中对“Blocks”的操作一致。

![image-20220629113410620](https://cdn.jsdelivr.net/gh/kun-k/blogweb/imageimage-20220629113410620.png)

