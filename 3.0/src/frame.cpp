#include "myslam/frame.h"

namespace myslam
{
Frame::Frame()
:id_(-1),time_stamp_(-1),camera_(nullptr)
{

}
Frame::Frame(long id,double time_stamp ,SE3 T_c_w,Camera::Ptr camera,Mat color, Mat depth)
:id_(id),time_stamp_(time_stamp),T_c_w_(T_c_w),camera_(camera),color_(color),depth_(depth)
{

}

Frame::Ptr Frame::creatFrame()
{
    static long factory_id = 0;
    return Frame::Ptr(new Frame(factory_id++));
}

double Frame::findDepth(const cv::KeyPoint& kp)
{
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    ushort d = depth_.ptr<ushort>(y)[x]; //深度图中xy点的深度
    //如果深度不为0，则直接返回；如果深度为0，则返回周围深度不为0的点；
    if (d!=0)
    {
        return double(d)/camera_->depth_scale_;
    }
    else
    {
        //检查周围的点
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for (int i = 0; i < 4; i++)
        {
            d = depth_.ptr<ushort>(y+dy[i])[x+dx[i]];
            if (d!=0)
            {
                return double(d)/camera_->depth_scale_;
            }
            
        }
    }
    return -1.0;    
}

Vector3d Frame::getCamCenter() const
{
    // 返回相机光芯
    return T_c_w_.inverse().translation(); 
}

bool Frame::isInFrame(const Vector3d& pt_world)
{
    /*
    判断世界坐标系中的点是否在一个帧中，先求出该点转换到相机坐标系中的位置
    如果深度<0 则返回否
    否则 求出像素坐标中的点
    */
    Vector3d p_cam = camera_->world2camera(pt_world,T_c_w_);
    if (p_cam(2,0) < 0)
    {
        return false;
    }
    else
    {
        Vector2d pixel = camera_->world2pixel(pt_world,T_c_w_);
        return pixel(0,0)> 0 && pixel(1,0) > 0
            && pixel(0,0) < color_.cols
            && pixel(1,0) < color_.rows;
    }    
}

}
