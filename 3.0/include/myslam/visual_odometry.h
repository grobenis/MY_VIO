#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H
/*
两两帧间的特征匹配关系

流程：
1. 对新来的当前帧，提取关键点和描述子。
2. 如果系统未初始化，以该帧为参考帧，根据深度图计算关键点的 3D 位置，返回1。
3. 估计参考帧与当前帧间的运动。
4. 判断上述估计是否成功。
5. 若成功，把当前帧作为新的参考帧，回 1。
6. 若失败，计算连续丢失帧数。
当连续丢失超过一定帧数，置 VO 状态为丢失，算法结束。若未超过，返回 

本文件将给出该算法的实现
*/

#include "common_include.h"
#include "frame.h"
#include "map.h"
#include "mappoint.h"

namespace myslam
{

class VisualOdometry
{
public :
    typedef shared_ptr<VisualOdometry> Ptr; //视觉里程计的指针
    enum VOState {
        INITIALIZING = -1,
        OK = 0,
        LOST
    }; //枚举视觉里程计的状态;

    // 该版本的视觉里程计只考虑两帧之间的运动
    VOState state_; //当前的视觉里程计指针
    Map::Ptr map_; //由所有帧建立的帧和路标点

    Frame::Ptr ref_; //参考帧
    Frame::Ptr curr_; //当前帧
    
    cv::Ptr<cv::ORB> orb_; //orb 检测子
    // vector<cv::Point3d> pts_3d_ref_; // -参考帧中的关键点的3d位置 3.0版本中进行移除
    vector<cv::KeyPoint> keypoints_curr_; //当前帧的关键点
    Mat descriptors_curr_; //当前帧的描述子

    cv::FlannBasedMatcher   matcher_flann_;     // + flann matcher flann匹配子
    vector<MapPoint::Ptr>   match_3dpts_;       // + matched 3d points 匹配的3d像素
    vector<int>             match_2dkp_index_;  // + matched 2d pixels (index of kp_curr) 匹配的2d像素
   
    // Mat descriptors_ref_; // - 参考帧的描述子 
    // vector<cv::DMatch> feature_matches_; // - 特征匹配  
    // - 直接与地图比较 因此不用再与参考帧比较

    // SE3 T_c_r_estimated_; // - 参考帧的估计pose

    SE3 T_c_w_estimated_;  // 相机在世界坐标系下的转换矩阵
    int num_inliers_; //icp中非线性特征的数目
    int num_lost_; //丢失次数

    //参数（8个）
    int num_of_features_; //特征数目
    double scale_factor_; // 缩放因子
    int level_pyramid_; //金字塔的层数
    float match_ratio_; //选择好的匹配点对的比例
    int max_num_lost_; //丢失的最大数目
    int min_inliers_; //最小的正常值
    double key_frame_min_rot; //两个关键帧之间的最小旋转
    double key_frame_min_trans; //两个关键帧之间的最小平移

    double  map_point_erase_ratio_; // 看不见的点在地图中的比例

public://函数
    VisualOdometry();
    ~VisualOdometry();
    
    bool addFrame(Frame::Ptr frame); //添加一个新的帧

protected:
    // 内部操作
    void extractKeyPoints(); //提取特征点
    void computeDescriptors(); //计算描述子
    void featureMatching(); //特征匹配
    void poseEstimationPnP(); //PnP姿态估计
    // - void setRef3DPoints(); //设置参考帧的3D点

    // 对地图进行优化的函数，删除不在视野内的点以及匹配数量时减少时添加新点等
    void optimizeMap();    
    // + 加入新的地图点
    void addMapPoints();
    
    void addKeyFrame(); //添加关键帧    
    bool checkEstimatedPose(); //检查估计姿势
    bool checkKeyFrame(); //检查关键帧

    // + 获得视觉角度信息
    double getViewAngle(Frame::Ptr frame,MapPoint::Ptr point);

};
}
#endif //VISUAL_ODOMETRY