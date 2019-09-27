#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include "myslam/g2o_types.h"


namespace myslam
{

// 视觉里程计的构造函数
VisualOdometry::VisualOdometry() :
state_( INITIALIZING ),ref_(nullptr),curr_(nullptr),map_(new Map),num_lost_(0),num_inliers_(0)
{
    num_of_features_ = Config::get<int> ("number_of_features");
    scale_factor_ = Config::get<double> ("scale_factor");
    level_pyramid_ = Config::get<int> ("level_pyramid");
    match_ratio_ = Config::get <float> ("match_ratio");
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

void VisualOdometry::extractKeyPoints()
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
    // 2.0:在两帧之间匹配特征 使用OPENCV对curr和ref提取匹配
    // 3.0:匹配之前，我们从地图中拿出一些候选点（出现在视野内的点），
    //     然后将它们与当前帧的特征描述子进行匹配。

    boost::timer timer; // 3.0 + 用来计算匹配速度
    vector<cv::DMatch> matches;

    // 选出一些候选点来
    Mat desp_map;
    vector<MapPoint::Ptr> candidates;
    for (auto& allpoints:map_->map_points_)
    {
        MapPoint::Ptr& p = allpoints.second;
        // 检查p点是否在当前帧
        if (curr_->isInFrame(p->pos_))
        {
            p->visible_times_++;
            candidates.push_back(p);
            desp_map.push_back(p->descriptor_);
        }        
    }

    matcher_flann_.match(desp_map,descriptors_curr_,matches);
    
    // 选择其中最好的匹配点
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    match_3dpts_.clear();
    match_2dkp_index_.clear();

    for ( cv::DMatch& m:matches )
    {
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            match_3dpts_.push_back(candidates[m.queryIdx]);
            match_2dkp_index_.push_back(m.trainIdx);
        }
    }
    cout<<"good matches: "<<match_3dpts_.size()<<endl;
    cout<<"match cost time: "<<timer.elapsed<<endl;
}

void VisualOdometry::poseEstimationPnP() //2.0版本主要修改了此函数
{
    // 构建3D-2D的观察矩阵
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;

    for (cv::DMatch m:feature_matches_)
    {
        pts3d.push_back( pts_3d_ref_[m.queryIdx]);
        pts2d.push_back( keypoints_curr_[m.trainIdx].pt);
    }

    Mat K = ( cv::Mat_<double>(3,3) <<
        ref_->camera_->fx_, 0, ref_->camera_->cx_, 0,
        ref_->camera_->fy_, ref_->camera_->cy_,
        0,0,1
    );

    // 相比1.0 优化了PnP的结果
    Mat rvec, tvec, inliers;
    cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
    //cout << inliers.cols << endl;
    num_inliers_ = inliers.rows;
    cout << "pnp inliers: " << num_inliers_ << endl;
    T_c_r_estimated_ = SE3(
        SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)), 
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );

    //以使用RANSAN PnP得到的值作为初值，让后使用干 g2o 进行BA优化 来优化得到的姿态
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(T_c_r_estimated_.rotation_matrix(),T_c_r_estimated_.translation()));
    optimizer.addVertex(pose);

    //优化边
    for (int i = 0; i < inliers.rows; i++)
    {
        int index = inliers.at<int>(i,0);

        EdgePXYZ2UVPoseOnly* edge = new EdgePXYZ2UVPoseOnly();
        edge->setId(i);
        edge->setVertex(0,pose);
        edge->camera_ = curr_->camera_.get();
        edge->point_ = Vector3d (pts3d[index].x,pts3d[index].y,pts3d[index].z);
        edge->setMeasurement( Vector2d(pts2d[index].x, pts2d[index].y) );
        edge->setInformation( Eigen::Matrix2d::Identity() ); //设置测量值为第?帧下的相机归一化平面坐标
        optimizer.addEdge(edge); 
    }
    optimizer.initializeOptimization();
    optimizer.optimize(10);// 迭代十轮

    T_c_r_estimated_ = SE3(
        pose->estimate().rotation(),
        pose->estimate().translation()    
    );
}

void VisualOdometry::setRef3DPoints()
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

void VisualOdometry::optimizeMap()
{
    // 移除看不见的点
    for (auto i = map_->map_points_.begin();i != map_->map_points_.end();)
    {
        if (!curr_->isInFrame(i->second->pos_)) //判断该点是否在当前帧中 如果不在 则移除
        {
            i = map_->map_points_.erase(i); // 抹除该点
            continue;
        }
        // 匹配率 移除匹配不好的点
        float match_ratio = float(i->second->matched_times_)/i->second->visible_times_;
        if( match_ratio < map_point_erase_ratio_)
        {
            i = map_ -> map_points_.erase(i);
            continue;
        }
        // 获取视觉角度
        double angle = getViewAngle(curr_,i->second);
        if ( angle > M_PI/6 )
        {
            i = map_->map_points_.erase(i);
            continue;
        }
        
        if (i->second->good_==false)
        {
            //TODO 试图三角化这个地图点
        }
        i++;
    }

    if (match_2dkp_index_.size()<100)
    {
        addMapPoints();
    }
    if (map_->map_points_.size()>1000)
    {
        // TODO 地图太大了 不好维护 因此要移除一些
        map_point_erase_ratio_ += 0.5;
    }
    else
    {
        map_point_erase_ratio_ = 0.1;
    }
    cout<<"map_points: "<< map_->map_points_.size()<<endl;
        
    
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
    if ( rot.norm() >key_frame_min_rot || trans.norm() > key_frame_min_trans )
        return true;
    return false;
}

void VisualOdometry::addKeyFrame()
{   
    // + 提取第一帧的特征点号将第一帧的所有特征点全部加入地图中 
    if (map_->keyframes_.empty())
    {
        for (size_t i = 0; i < keypoints_curr_.size(); i++)
        {
            double d = curr_->findDepth(keypoints_curr_[i]);
            if(d<0)
                continue;
            // 要得到像素在世界坐标系下的点 首先要得到下相机坐标系下的点使用camera2world x y d
            // 然后利用camera——》world得到世界坐标系下的点
            Vector3d p_world = ref_->camera_->pixel2world(
                Vector2d( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y),curr_->T_c_w_,d
            );

            Vector3d n = p_world - ref_->getCamCenter();
            n.normalize();
            // 正交化后就可以将新的地图点加入地图了
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                p_world,n,descriptors_curr_.row(i).clone(),curr_.get()
            );
            map_->insertMapPoint(map_point);
        }        
    }    
    cout << "adding a key-frame" << endl;
    map_ -> insertKeyFrames ( curr_ );
    ref_ = curr_;
}

}