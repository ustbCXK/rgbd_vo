/*
 * @Descripttion: file content
 * @version: 
 * @Author: congsir
 * @Date: 2021-07-05 10:28:22
 * @LastEditors: Do not edit
 * @LastEditTime: 2021-07-05 10:30:18
 */
#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include "myslam/common_include.h"
#include "camera.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

namespace myslam
{
class EdgeProjectXYZRGBD : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void computeError();
    virtual void linearizeOplus();
    virtual bool read( std::istream& in ){}
    virtual bool write( std::ostream& out) const {}
    
};

// only to optimize the pose, no point
class EdgeProjectXYZRGBDPoseOnly: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Error: measure = R*point+t
    virtual void computeError();
    virtual void linearizeOplus();
    
    virtual bool read( std::istream& in ){}
    virtual bool write( std::ostream& out) const {}
    
    Vector3d point_;
};

class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //还是边类型定义中最核心的老两样：
    //误差计算函数，实现误差计算方法
    virtual void computeError();
    //线性增量函数，也就是计算雅克比矩阵的方法
    virtual void linearizeOplus();
    
    //读写功能函数，没用到只是定义了
    virtual bool read( std::istream& in ){}
    virtual bool write(std::ostream& os) const {};
    
    //把三维点和相机模型写成成员变量
    //方便误差计算和J计算
    Vector3d point_;
    Camera* camera_;
};

}


#endif // MYSLAM_G2O_TYPES_H
