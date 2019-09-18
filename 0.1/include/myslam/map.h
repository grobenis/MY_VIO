#ifndef MAP_H
#define MAP_H

#include "common_include.h"
#include "frame.h"
#include "mappoint.h"

class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    unordered_map<unsigned long , MapPoint::Ptr> map_points_; // all landmarks
    unordered_map<unsigned long , Frame::Ptr> keyframes_; //帧的指针

public:
    Map(/* args */);
    void insertKeyFrames(Frame::Ptr frame);
    void insertMapPoint(MapPoint::Ptr map_point );
    ~Map();
};

#endif