#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "common_include.h"
#include "frame.h"
/* 
该类路标点,我们将估计它的世界坐标，并且我们会拿当前帧提取到的特
征点与地图中的路标点匹配，来估计相机的运动，因此还需要存储它对应
的描述子。此外，我们会记录一个点被观测到的次数和被匹配到的次数，
作为评价它的好坏程度的指标。
*/

/*

3.0
地图又可以分为局部（Local）地图和全局（Global）地图两种，由于用
途不同，往往分开讨论。顾名思义，局部地图描述了附近的特征点信息——
我们只保留离相机当前位置较近的特征点，而把远的或视野外的特征点丢掉。
这些特征点是用来和当前帧匹配来求相机位置的，所以我们希望它能够做的
比较快。另一方面，全局地图则记录了从 SLAM 运行以来的所有特征点。它
显然规模要大一些，主要用来表达整个环境，但是直接在全局地图上定位，对
计算机的负担就太大了。它主要用于回环检测和地图表达。

*/


namespace myslam
{

class MapPoint
{
public:
    typedef shared_ptr<MapPoint> Ptr; 

    unsigned long id_;   //路标点的ID

    static unsigned long factory_id_; //factory id 因子id
    bool good_; //是否是一个好的地图点
    
    Vector3d pos_; //世界坐标系中的位置
    Vector3d norm_; //视线方向的法线
    Mat descriptor_; // 匹配的描述子

    list<Frame*> observed_fames_; //可以看见该点的帧

    int matched_times_; //姿态估计
    int visible_times_; //在当前帧出现的次数

    // int observed_times_; //特征匹配算法看到的次数
    // int correct_times_; //姿态估计的 being an inliner in pose estimation

    /* data */
    
public:
    MapPoint();
    MapPoint(
        unsigned long id, 
        const Vector3d& position,
        const Vector3d& norm,
        Frame* frame = nullptr,
        const Mat& descriptor = Mat()
    );// Mappoint 由ID、位置 法线，帧，描述子确定

    inline cv::Point3f getPositionCV() const{
        return cv::Point3f(pos_(0,0),pos_(1,0),pos_(2,0));
    }

    // 创建Mappoint的两种方式
    static MapPoint::Ptr createMapPoint();
    static MapPoint::Ptr createMapPoint(
        const Vector3d& pos_world,
        const Vector3d& norm,
        const Mat& descriptor,
        Frame* frame
    );

    ~MapPoint(){};
};

}
#endif //MAPPOINT_H