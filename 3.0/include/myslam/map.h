#ifndef MAP_H
#define MAP_H

#include "common_include.h"
#include "myslam/frame.h"
#include "mappoint.h"

namespace myslam
{

class Map
{
/*
    Map 类中实际存储了各个关键帧和路标点，
    既需要随机访问，又需要随时插入和删除，
    因此我们使用散列（Hash）来存储它们。
*/
public:
    typedef shared_ptr<Map> Ptr;
    unordered_map<unsigned long , MapPoint::Ptr> map_points_; // all landmarks
    unordered_map<unsigned long , Frame::Ptr> keyframes_; //帧的指针

public:
    Map(){};

    void insertKeyFrames(Frame::Ptr frame); // 插入关键帧
    void insertMapPoint(MapPoint::Ptr map_point ); // 插入地图特征点

    ~Map(){};
};

}
#endif //MAP_H