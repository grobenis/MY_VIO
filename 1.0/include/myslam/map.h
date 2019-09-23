#ifndef MAP_H
#define MAP_H

#include "common_include.h"
#include "myslam/frame.h"
#include "mappoint.h"

namespace myslam
{

class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    unordered_map<unsigned long , MapPoint::Ptr> map_points_; // all landmarks
    unordered_map<unsigned long , Frame::Ptr> keyframes_; //帧的指针

public:
    Map(){};
    void insertKeyFrames(Frame::Ptr frame);
    void insertMapPoint(MapPoint::Ptr map_point );
    ~Map(){};
};

}
#endif