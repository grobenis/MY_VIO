#ifndef MAP_H
#define MAP_H

#include "common_include.h"
#include "frame.h"

class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    unordered_map<unsigned long , MapPoint::Ptr> map_points; // all landmarks
    unordered_map<unsigned long , Frame::Ptr> 

public:
    Map(/* args */);
    ~Map();
};

#endif