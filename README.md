# MY_VIO

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

## 0.1 版本
共5个类
1. Frame 
2. Camera
3. MapPonit
4. Map
5. Config