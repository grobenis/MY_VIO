#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "../include/myslam/visual_odometry.h"
#include "../include/myslam/config.h"

namespace myspace
{

// 视觉里程计的构造函数
VisualOdometry::VisualOdometry() :
state_(INITIALIZING),ref_(nullptr),curr_(nullptr),map_(new Map),num_lost_(0),num_inliers_(0)
{
    num_of_features_ = Config::get<int> ("number_of_features");
    scale_factor_ = Config::get<double> ("scale_factor");
    level_pyramid_ = Config::get<double> ("level_pyramid");
    match_ratio_ = Config::get <float> ("match_ratio")；
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
}

// 析构函数
VisualOdometry::~VisualOdometry()
{

}

bool VisualOdometry::addFrame(Frame::Ptr frame)
{
    // VO有三种状态：初始化，正常，丢失
    switch (state_)
    {
    case INITIALIZING:
    {   
        //若VO为初始化 把到来的帧加入视觉里程计中, 
        //要修改的地方，VO状态，当前帧与参考帧，地图，提取特征，计算描述子
        state_ = OK;
        curr_ = ref_ = frame;
        map_ ->insertKeyFrames(frame);

        // 从第一帧中提取特征并计算描述子
        extractKeyPoints();
        computeDescriptors();
        
        // 根据深度图计算关键点的3D位置点
        setRef3DPoints();
        break;
    }
    case OK:
    {
        // 如果状态正常的话，
        curr_ = frame;
        extractKeyPoints();
        computeDescriptors();
        featureMatching();
        poseEstimationPnP();
        if (checkEstimatedPose()==true)
        {
            curr_->T_c_w_ = T_c_r_estimated_ *ref_->T_c_w_;
            ref_ = curr_;
            setRef3DPoints();
            num_lost_ = 0;
            if (checkKeyFrame() == true)
            {
                addKeyFrame();
            }            
        }
        else
        {
            num_lost_++;
            if (num_lost_ > max_num_lost_)
            {
                state_ = LOST;
            }
            return false;
        }      
        break;
    }
        
    case LOST:
    {
        cout<<"VO has lost"<<endl;
        break;
    }
    }
    return true;
}

VisualOdometry::extractKeyPoints()
{
    // 利用orb方法提取当前帧的特征点
    orb_ -> detect ( curr_->color_, keypoints_curr_ ); //orb提取特征
}

void VisualOdometry::computeDescriptors()
{
    // 计算当前帧描述子
    orb_ -> compute ( curr_->color_, keypoints_curr_, descriptors_curr_);
}

void VisualOdometry::featureMatching()
{
    // 在两帧之间匹配特征点
    // 使用OPENCV对curr和ref提取匹配
    vector<cv::DMatch> matches;
    cv::BFMatcher matcher ( cv::NORM_HAMMING );
    matcher.match ( descriptors_ref_, descriptors_curr_, matches );

    // 选择其中最好的匹配点
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    feature_matches_.clear();
    for ( cv::DMatch& m : matches )
    {
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            feature_matches_.push_back(m);
        }
    }
    cout<<"good matches: "<<feature_matches_.size()<<endl;    
}

void VisualOdemetry::setRef3DPoints()
{
    // 根据深度图计算关键点的3D位置
    // select the features with depth measurements 
    pts_3d_ref_.clear();
    descriptors_ref_ = Mat();
    for ( size_t i=0; i<keypoints_curr_.size(); i++ )
    {
        double d = ref_->findDepth(keypoints_curr_[i]);               
        if ( d > 0)
        {
            Vector3d p_cam = ref_->camera_->pixel2camera(
                Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d
            );
            pts_3d_ref_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
            descriptors_ref_.push_back(descriptors_curr_.row(i));
        }
    }  
}

void VisualOdometry::poseEstimationPnP()
{
    // 构建3D-2D的观察矩阵
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;

    for (cv::DMatch m::feature_matches_)
    {
        pts3d.push_back( pts_3d_ref[m.queryIdx]);
        pts2d.push_back( keypoints_curr_[m.trainIdx].pt);
    }

    Mat K = ( cv::Mat_<double> <<
        ref_->camera_->fx_, 0, ref_->camera_->cx_,0,
        ref_->camera_->fy_, ref_->camera_->cy_,0,0,1
    );

    Mat rvec,tvec,inliers;
    /*
    bool solvePnPRansac( InputArray objectPoints, InputArray imagePoints,
                        InputArray cameraMatrix, InputArray distCoeffs,
                        OutputArray rvec, OutputArray tvec,
                        bool useExtrinsicGuess = false, int iterationsCount = 100,
                        float reprojectionError = 8.0, double confidence = 0.99,
                        OutputArray inliers = noArray(), int flags = SOLVEPNP_ITERATIVE );

    */
    cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
    num_inliers_ = inliers.rows;
    cout << "pnp inliers: " << num_inliers_ << endl;
    T_c_r_estimated_ = SE3(
        SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)), 
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );
}

bool VisualOdometry::checkEstimatedPose()
{
    // check if the estimated pose is good
    // inliers：正常值
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    Sophus::Vector6d d = T_c_r_estimated_.log();
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm()<<endl;
        return false;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame()
{
    Sophus::Vector6d d = T_c_r_estimated_.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
}

void VisualOdometry::addKeyFrame()
{
    cout << "adding a key-frame" << endl;
    map_ -> insertKeyFrame ( curr_ );
}

}