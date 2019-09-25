/*

接下来，我们沿着之前的内容，尝试一些改进 VO 的方法。
本节中，我们来尝试RANSAC PnP 加上迭代优化的方式估计相机位姿，
看看是否对前一节的效果有所改进。
非线性优化问题的求解，已经在第六、七讲介绍过了。
由于本节的目标是估计位姿而非结构，我们以相机位姿 ξ 为优化变量，
通过最小化重投影误差，来构建优化问题。与之前一样，
我们自定义一个 g2o 中的优化边。它只优化一个位姿，
因此是一个一元边

*/
#ifndef G2O_TYPES_H
#define G2O_TYPES_H

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_vertex.hpp>
#include <g2o/core/base_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>

#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include "common_include.h"
#include "camera.h"

namespace myslam
{

class EdgePXYZ2UVPoseOnly:public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW //动态变量(例如Eigen::VectorXd)会动态分配内存，因此会自动地进行内存对齐。

    virtual void computeError();
    virtual void linearOplus();

    virtual bool read( std::istream& in){};
    virtual bool write( std::ostream& os)const{};
    //把三维点和相机模型放入它的成员变量中，方便计算重投影误差和雅克比矩阵

    Vector3d point_;
    Camera* camera_;
};

}
#endif //G2O_TYPES_H