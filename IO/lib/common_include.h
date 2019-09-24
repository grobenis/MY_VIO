#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

//定义了所有需要包含的库以避免每次包含很长的列表

//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

//Sophus
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SE3;
using Sophus::SO3;

//openCV
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
using cv::Mat;

//std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>

using namespace std;

#endif //COMMON_INCLUDE_H

