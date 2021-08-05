#include "myslam/g2o_types.h"

namespace myslam
{
void EdgeProjectXYZRGBD::computeError()
{
    const g2o::VertexSBAPointXYZ* point = static_cast<const g2o::VertexSBAPointXYZ*> ( _vertices[0] );
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[1] );
    _error = _measurement - pose->estimate().map ( point->estimate() );
}

void EdgeProjectXYZRGBD::linearizeOplus()
{
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *> ( _vertices[1] );
    g2o::SE3Quat T ( pose->estimate() );
    g2o::VertexSBAPointXYZ* point = static_cast<g2o::VertexSBAPointXYZ*> ( _vertices[0] );
    Eigen::Vector3d xyz = point->estimate();
    Eigen::Vector3d xyz_trans = T.map ( xyz );
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    _jacobianOplusXi = - T.rotation().toRotationMatrix();

    _jacobianOplusXj ( 0,0 ) = 0;
    _jacobianOplusXj ( 0,1 ) = -z;
    _jacobianOplusXj ( 0,2 ) = y;
    _jacobianOplusXj ( 0,3 ) = -1;
    _jacobianOplusXj ( 0,4 ) = 0;
    _jacobianOplusXj ( 0,5 ) = 0;

    _jacobianOplusXj ( 1,0 ) = z;
    _jacobianOplusXj ( 1,1 ) = 0;
    _jacobianOplusXj ( 1,2 ) = -x;
    _jacobianOplusXj ( 1,3 ) = 0;
    _jacobianOplusXj ( 1,4 ) = -1;
    _jacobianOplusXj ( 1,5 ) = 0;

    _jacobianOplusXj ( 2,0 ) = -y;
    _jacobianOplusXj ( 2,1 ) = x;
    _jacobianOplusXj ( 2,2 ) = 0;
    _jacobianOplusXj ( 2,3 ) = 0;
    _jacobianOplusXj ( 2,4 ) = 0;
    _jacobianOplusXj ( 2,5 ) = -1;
}

void EdgeProjectXYZRGBDPoseOnly::computeError()
{
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
    _error = _measurement - pose->estimate().map ( point_ );
}

void EdgeProjectXYZRGBDPoseOnly::linearizeOplus()
{
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
    g2o::SE3Quat T ( pose->estimate() );
    Vector3d xyz_trans = T.map ( point_ );
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    _jacobianOplusXi ( 0,0 ) = 0;
    _jacobianOplusXi ( 0,1 ) = -z;
    _jacobianOplusXi ( 0,2 ) = y;
    _jacobianOplusXi ( 0,3 ) = -1;
    _jacobianOplusXi ( 0,4 ) = 0;
    _jacobianOplusXi ( 0,5 ) = 0;

    _jacobianOplusXi ( 1,0 ) = z;
    _jacobianOplusXi ( 1,1 ) = 0;
    _jacobianOplusXi ( 1,2 ) = -x;
    _jacobianOplusXi ( 1,3 ) = 0;
    _jacobianOplusXi ( 1,4 ) = -1;
    _jacobianOplusXi ( 1,5 ) = 0;

    _jacobianOplusXi ( 2,0 ) = -y;
    _jacobianOplusXi ( 2,1 ) = x;
    _jacobianOplusXi ( 2,2 ) = 0;
    _jacobianOplusXi ( 2,3 ) = 0;
    _jacobianOplusXi ( 2,4 ) = 0;
    _jacobianOplusXi ( 2,5 ) = -1;
}

// 重投影误差
void EdgeProjectXYZ2UVPoseOnly::computeError()
{
    //顶点数组中取出顶点，转换成位姿指针类型，
    //其实左边的pose类型可以写成auto
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
    //误差计算，测量值减去估计值，也就是重投影误差
    //估计值的计算式T*P，得到相机坐标系下的坐标，
    //然后利用camera2pixel得到像素坐标系下的坐标
    _error = _measurement - camera_->camera2pixel ( 
        pose->estimate().map(point_) );
}


void EdgeProjectXYZ2UVPoseOnly::linearizeOplus()
{
    /*整体思路：
    重投影误差的雅克比矩阵在P164页中式7.45呈现了，直接构造
    构造时发现 需要变换后的空间点坐标。所以求出
    */

    //从顶点求位姿
    g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
    //由位姿构造一个四元数形式T
    g2o::SE3Quat T ( pose->estimate() );
    //用T求得变换后的3D点坐标 T*P
    Vector3d xyz_trans = T.map ( point_ );
    //变换后的3D点 xyz都有了
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double z_2 = z*z;

    _jacobianOplusXi ( 0,0 ) =  x*y/z_2 *camera_->fx_;
    _jacobianOplusXi ( 0,1 ) = - ( 1+ ( x*x/z_2 ) ) *camera_->fx_;
    _jacobianOplusXi ( 0,2 ) = y/z * camera_->fx_;
    _jacobianOplusXi ( 0,3 ) = -1./z * camera_->fx_;
    _jacobianOplusXi ( 0,4 ) = 0;
    _jacobianOplusXi ( 0,5 ) = x/z_2 * camera_->fx_;

    _jacobianOplusXi ( 1,0 ) = ( 1+y*y/z_2 ) *camera_->fy_;
    _jacobianOplusXi ( 1,1 ) = -x*y/z_2 *camera_->fy_;
    _jacobianOplusXi ( 1,2 ) = -x/z *camera_->fy_;
    _jacobianOplusXi ( 1,3 ) = 0;
    _jacobianOplusXi ( 1,4 ) = -1./z *camera_->fy_;
    _jacobianOplusXi ( 1,5 ) = y/z_2 *camera_->fy_;
}


}
