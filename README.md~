# karto_slam_with_loop_closing
Author：Robin luo
2017/11/12 
这个项目是利用orbslam视觉闭环（闭环、重定位（词袋模型）），为激光提供真闭环信息

设备：北阳激光、turtlebot、相机

激光hokuyo
http://wiki.ros.org/hokuyo_node 配置下就可以，该scan的Frame是base_laser
这里我们这里最好指定base_link与base_laser的tf关系，方便录包使用，也可以
选择在slam_karto的 launch文件中进行设定
<node pkg="tf" type="static_transform_publisher" name="link_to_laser" args="0 0 0 0 0 0 1 /base_link /base_laser 10" />

turtlbot
提供里程计信息 odom Frame 到 base_link frame 的关系
启动 roslaunch kobuki_node minimal.launch
键盘控制roslaunch kobuki_keyop keyop.launch

karto slam 里面有两个包主要提供map Frame 与 odom Frame的关系关系
open_karto 是核心底层代码
slam_karto ros的上层控制与底层open——karto交互，karto::LocalizedRangeScan* range_scan 通过这个类，
我们可以添加自己的一下数据与底层进行交互。
核心代码入口在  if((processed = mapper_->Process(range_scan)))

在这里遇到里坐标的问题，karto_slam正常工作坐标关系必须是map-odom-base_link-base_laser，
可以利用 rosrun tf view_frames
注意不同的底盘和激光后面两个Frame名也可能不一样，需要自行修改
<param name="odom_frame" value="odom_combined"/>改
<param name="odom_frame" value="odom"/>

这里加闭环采用的是订阅的形式，写在上层，然后送进底层，在if (m_pDoLoopClosing->GetValue()）加约束
这里为了方便测试，这里我们用了一个简单的发布topic进行测试  

代码稍微接卸一下，完全来自于我的师兄，师兄棒棒的，精通各种slam
http://blog.csdn.net/qq_31785865/article/details/53452853链接如下

应用层
作为用户需要知道kartoslam是如何使用的。首先需要从github上git以下两个包：
git clone https://github.com/ros-perception/open_karto（开源的karto包，实现底层的kartoslam）
git clone https://github.com/ros-perception/slam_karto（ros层，也就是应用层的kartoslam接口）
将这些包放到工作目录，完成编译，我们就能尝试运行代码啦（当然你会发现，运行会有error）~~~~~~~
因为，在这之前我们需要为这个kartoslam定义一个tf变换关系才能保证代码的正确运行：
在karto_slam.launch中加入/base_link与laser的相对/tf关系（<node pkg="tf" type="static_transform_publisher" name="link_to_laser" args="0 0 0 0 0 0 1 /base_link /laser 10" />）（为什么呢？后面再讲～～）
如果你有一个激光雷达，一台turtlebot，还有一台电脑，那么你就可以利用kartoslam实现slam啦!当然，我们也可以在karto_slam.launch修改一下配置（比方说，我们希望的地图更新频率：map_update_interval，地图的分辨率：resolution等等）。
理论层
机器人在运行slam的过程中，需要知道几个参考坐标系之间的关系：
map_frame：世界坐标系中的固定frame。也就是说，这是一个全局的固定frame，它理论上不会随着机器人的移动或者测量的误差而产生漂移，一般设置系统的初始状态作为坐标原点。
odom_frame：里程计frame。它是移动机器人在移动过程中利用里程计（如车轮编码器，视觉里程计等）对机器人进行位姿估计的坐标系，一般设置机器人初始位置作为坐标原点，总所周知，里程计在长时间的使用中会出现一下累计误差。
baselink_link_frame:机器人frame。它是机器人坐标系，是一个移动的坐标系。
laser_frame:激光frame。它是激光雷达的坐标系，固定在机器人上，与baselink_link_frame往往只存在一个平移变换量。
这4个坐标系是按照树状结构组织起来的，它们的对应关系如下图所示：

从frame图中可以看到，laser_frame和aselink_link_frame的关系是由/link_to_laser得到的（也就是一开始我们在launch中添加的代码）；baselink_link_frame和odom_frame之间的关系是由底层的里程计信息topic输出得到的；而odom_frame与map_frame之间的变换关系是由karto_slam得到的。karto_slam一直在做的事情就是：不断去修正里程计与固定的全局坐标系map_frame的误差，从而得到机器人相对于固定的世界坐标系更加精准的位姿信息。
代码层
代码从slam_karto.cpp开始，首先需要有接口函数
[cpp] view plain copy
SlamKarto::addScan(karto::LaserRangeFinder* laser,const sensor_msgs::LaserScan::ConstPtr& scan,  karto::Pose2& karto_pose)  
得到里程计和激光信息，并通过
[cpp] view plain copy
  // create localized range scan  
  karto::LocalizedRangeScan* range_scan = new karto::LocalizedRangeScan(laser->GetName(), readings);  
range_scan->SetOdometricPose(karto_pose);  
  range_scan->SetCorrectedPose(karto_pose);  
将数据放入kato特定的数据结构当中。
[cpp] view plain copy
(processed = mapper_-><span style="color:#003333;">Process</span>(range_scan))  
Process是karto的入口函数，首先需要初始化：
[cpp] view plain copy
if (m_Initialized == false)  
{  
  // initialize mapper with range threshold from device  
  Initialize(pLaserRangeFinder->GetRangeThreshold());  
}  

之后对满足关键帧要求的数据进行匹配
[cpp] view plain copy
m_pSequentialScanMatcher->MatchScan(pScan,  
                                            m_pMapperSensorManager->GetRunningScans(pScan->GetSensorName()),  
                                                                                    bestPose,  
                                                                                    covariance);  
得到估计的位姿信息bestPose和对应的协方差矩阵convariance。
至于匹配的方法，具体就不细讲了。主要用的是map和frame的匹配，采用窗口法寻找最优的变换矩阵，并采用粗匹配（coarseSearch），精匹配（fineSearch）和建立一个lookupTable加快了寻优的速度，提高寻优精度。
之后将当前帧放入图优优化中，增加vertex和edges。
注：图的构造（感觉自己说的好绕口）
1, 只提取关键帧（两帧直接距离大于设定值） 
2，当前帧相邻时间的关键帧相连 
3，当前帧与runningScan中最近的帧相连 (runningScan可以理解为局部地图)
4，与当前帧相邻但不相连的帧链中最近的帧相连（闭环连接关系） 
5，闭环检测：将在一定范围内的不与当前帧相连且不与NearLinkedScans相连的帧连接起来组成闭环（也就是在一定范围内不与当前帧有任何连线的帧连接起来，构成闭环）
然后，尝试闭环检测
[cpp] view plain copy
if (m_pDoLoopClosing->GetValue())  
{  
  std::vector<Name> deviceNames = m_pMapperSensorManager->GetSensorNames();  
  const_forEach(std::vector<Name>, &deviceNames)  
  {  
    m_pGraph->TryCloseLoop(pScan, *iter);  
  }  
}  
最后更新corrected_pose，publish map_to_odom的tf关系（也就是里程计的漂移误差）。
参考文献
《Efficient Sparse Pose Adjustment for 2D Mapping》

