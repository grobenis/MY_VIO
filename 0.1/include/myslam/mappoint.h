#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "common_include.h"
/* 该类路标点,我们将估计它的世界坐标，并且我们会拿当前帧提取到的特
征点与地图中的路标点匹配，来估计相机的运动，因此还需要存储它对应的描述子。此外，
我们会记录一个点被观测到的次数和被匹配到的次数，作为评价它的好坏程度的指标。
*/
class MapPoint
{
public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long id_;   //路标点的ID
    Vector3d pos_; //世界坐标系中的位置
    Vector3d norm_; //视线方向的法线
    Mat descriptor_; // 匹配的描述子
    int observed_times_; //特征匹配算法看到的次数
    int correct_times_; //姿态估计的 being an inliner in pose estimation

    /* data */
public:
    MapPoint();
    MapPoint(long id, Vector3d position,Vector3d norm);

    static MapPoint::Ptr createMapPoint();
    ~MapPoint();
};
#endif MAPPOINT_H