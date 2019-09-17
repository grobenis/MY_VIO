#include "../include/myslam/visual_odometry.h"
#include "../include/myslam/config.h"

VisualOdometry::VisualOdometry():
state_(INITIALIZING),ref_(nullptr),curr_(nullptr),map_(new Map),num_lost_(0),num_inliers_(0)
{
    num_of_features_ = Config::
}
//VisualOdemetry::VisualOdometry();