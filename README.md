# MY_VIO
## 目的：实现一个适用于室内环境的双目相机和惯性测量单元的 setero_VIO

## 基本要求
1. 可以看到轨迹(使用viz模块)
2. 有效检测回环
3. 基于紧耦合的融合方式

## 目录说明
1. 1.0 版本：实现了最基本的视觉里程计；
2. 2.0 版本：完善视觉里程计，使用g2o优化，提高精度；
3. 3.0 版本：建立局部地图，使用pcl库画出地图，绘制出坐标点的轨迹；
4. IO ：使用单独的IMU实现位置姿态解算；


# 大致结构
## 目录结构
1. bin 用来存放可执行的二进制；
2. include/myslam 存放 slam 模块的头文件，主要是.h。这种做法的理由是，当你把包含目录设到 include 时，在引用自己的头文件时，需要写 include ”myslam/xxx.h”，这样不容易和别的库混淆。
3. src 存放源代码文件，主要是 cpp；
4. test 存放测试用的文件，也是 cpp；
5. lib 存放编译好的库文件；
6. config 存放配置文件；
7. cmake_modules 第三方库的 cmake 文件，在使用 g2o 之类的库中会用到它。

## 基本的数据结构
1. 帧
2. 路标
3. 配置文件
4. 坐标变换

## 包含的6个类
1. Frame 
2. Camera
3. MapPonit
4. Map
5. Config
6. visual_odometry

数据集采用：TUM数据集中的：rgbd_dataset_freiburg1_xyz
