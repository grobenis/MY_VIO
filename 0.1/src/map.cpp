#include "../myslam/map.h"
#include "../myslam/mappoint.h"

/*
    Map 类中实际存储了各个关键帧和路标点，既需要随机访问，又需要随时插入和删除，
    因此我们使用散列（Hash）来存储它们。
*/

namespace myslam
{
void Map::insertKeyFrames(Frame::Ptr frame)
{
    cout<<"key frame size = "<<keyframes_.size()<<endl;

    if (keyframes_.find(frame->id_)==keyframes_.end())
    {
        keyframes_.insert(make_pair(frame->id_,frame));
    }
    else
    {
        keyframes_[frame->id_] = frame ;
    }
}

void Map::insertMapPoint(MapPoint::Ptr map_point)
{
    if (map_points_.find(map_point->id_)==map_points_.end())
    {
        map_points_.insert(make_pair(map_point->id_,map_point));
    }
    else
    {
        map_points_[map_point->id_] = map_point;
    }
    
}
}