#ifndef FRAME_H
#define FRAME_H
#include "common_include.h"
#include "camera.h"

namespace myslam
{
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long id_; //该帧的id 
    double time_stamp_; //时间戳
    SE3 T_c_w_ ; //从世界坐标系到相机坐标系的转换矩阵
    Camera::Ptr camera_; //RGB_D 相机的针孔模型
    Mat color_, depth_; 

public: //数据成员
    Frame();
    Frame(long id,double time_stamp = 0,SE3 T_c_w_ = SE3(),Camera::Ptr camera = nullptr,Mat color = Mat(), Mat depth = Mat());

    ~Frame();

    // 构造函数
    static Frame::Ptr creatFrame();

    //找到深度图中的深度
    double findDepth(const cv::KeyPoint& kp);

    //得到相机中心
    Vector3d getCamCenter() const;

    //检测一个点是否属于这个帧
    bool isInFrame(const Vector3d& pt_world);

};
}
#endif //FRAME_H