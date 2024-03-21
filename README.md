# Fundamentals-of-autopilot-project

<p align="center">
  <a href="./README_en.md">English</a> |
  <a href="./README.md">简体中文</a>
</p>


本项目是一个基于MATLAB和simulink实现小车自动寻路与控制仿真的项目。

<img src="./仿真模型/结果展示/1--18.319.gif" alt="演示图片">

## 特性

- **路径规划**: 在系统生成的随机 2D 地图上，使用 A*算法实现起点至终点的可行路径规划。
- **平滑路径**: 设计缩放与插值策略，在加速路径规划的基础上，通过贝塞尔曲线插值法与 De Casteljau 算法对生成的可行路径进行优化，采用分段平滑化的方式平滑路径并处理碰撞问题。
- **无碰撞**: 利用 simulink 进行小车路径跟踪仿真，建立控制器仿真模型，成功使小车一次性无碰撞到达终点。
- **速度快**: 在竞速中获得 MATLAB 组前 3 名（总 35 人）。

## 安装

要运行此项目，您需要安装 MATLAB 以及 simulink工具箱，然后克隆此仓库：

```bash
git clone https://github.com/LinYujupiter/Fundamentals-of-autopilot-project.git
cd Fundamentals-of-autopilot-project
```

## 运行
您应该重点关注“仿真模型”文件夹，这是本项目主要的贡献与运行文件。

```bash
cd ./仿真模型
```

在“仿真模型”文件夹内各文件的功能如下：

- car_sim_for_student_r2015.slx是simulink的仿真模型文件。

- Generate_Miniature_Path.m是根据地图信息，将地图膨胀20像素，并缩小到0.2倍，然后生成路径的A*算法文件。此.m文件的输入为不同的sysu_standard.mat，输出为path.mat，里面包括缩小后地图的路径的x，y坐标。

- Smooth_Path.m是将缩小后的路径，放大，并经过多次平滑化和曲率筛选后，处理成适合车辆行驶的路径。然后转换这个路径的格式，将x,y等坐标信息变换坐标系，并添加phi和曲率信息，也就是处理成与示例文件traj_diySYSU.mat一样的格式文件并输出。

- lqr_nums.m是simulink仿真前的initfcn需要加载的文件。里面将地图和路径信息都load进工作区，并将lqr控制器的结果k也计算出来。

- make_GIF.m将仿真后保存下来的simout.mat文件解析，并通过函数复现出小车的行驶过程图，最后从中选取200帧左右做成GIF图。GIF的命名中的第二个数值即为此仿真运行时间（秒）。

- 这些文件中都有pathnums变量，这是为了命名方便使用的，代表了第几张地图的数据。例如，lqr_nums.m中如果pathnums='3'，则会在第三张地图上仿真。

运行顺序说明：MATLAB工作文件夹设置在仿真模型文件夹。首先将地图sysu_standard.mat命名好并放入地图与路径目录，然后把所有.m文件中的pathnums修改为命名的那个数字编号。然后先运行Generate_Miniature_Path.m，大约11000轮循环后会出结果。再运行Smooth_Path.m，然后进入simulink点运行，simulink运行完毕后，运行make_GIF.m，就能输出一个完整的GIF图。

## 开发

- **MATLAB**: 用于作为项目基础语言。
- **simulink**: 用于仿真小车运行。
- **LQR控制器**: 用于小车横向控制。
- **PID控制器**: 用于小车纵向控制。

## 贡献

我们欢迎任何形式的贡献，无论是新功能的提议、代码改进还是问题报告。请确保遵循最佳实践和代码风格指南。
