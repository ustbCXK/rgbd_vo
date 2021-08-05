/*
 * @Descripttion: file content
 * @version: 
 * @Author: congsir
 * @Date: 2021-07-02 10:40:38
 * @LastEditors: Do not edit
 * @LastEditTime: 2021-07-02 14:36:12
 */
#ifndef CAMERA_H
#define CAMERA_H
#include "myslam/common_include.h"

//命名空间----self！！！
namespace myslam{
    //RGBD的吧，比较简单

/**
 * @name: Camera类
 * @desc:   1、构造函数
 *          2、有参构造
 *          3、坐标变换函数
 * @param {*}
 * @return {*}
 */
class Camera
{
public:
    //智能指针定义一个Camera类型的指针，方便传参时直接使用
    typedef std::shared_ptr<Camera> Ptr;

    //1、相机的内参
    float fx_, fy_, cx_, cy_, depth_scale_;

    //相机构造函数
    Camera();
    //有参构造，利用初始化列表
    Camera(
        float fx_, 
        float fy_, 
        float cx_, 
        float cy_, 
        float depth_scale_=0
    ):fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale){}


    //相机坐标系的坐标变换,利用Eigen::Vector3d和Vector2d
    //三个坐标系：世界坐标系world、相机坐标系camera、成像坐标系pixel
    Vector3d world2camera ( const Vector3d& p_w, const SE3& T_c_w );//Tcw*Pw = Pc
    Vector3d camera2world ( const Vector3d& p_c, const SE3& T_c_w );//Twc*Pc = Pw
    
    // u_x = f_x * Xc/Zc + c_x
    // u_y = f_y * Yc/Zc + c_y
    Vector2d camera2pixel ( const Vector3d& p_c );                  
    Vector3d pixel2camera ( const Vector2d& p_p, double depth=1 ); 
    
    // pixel2world = pixel2camera -> camera2world
    Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth=1 );
    // world2pixel = world2camera -> camera2pixel
    Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w );
};
}
#endif //CAMERA_H