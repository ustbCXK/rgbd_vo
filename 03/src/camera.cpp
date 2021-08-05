/*
 * @Descripttion: file content
 * @version: 
 * @Author: congsir
 * @Date: 2021-07-02 11:53:29
 * @LastEditors: Do not edit
 * @LastEditTime: 2021-07-02 12:01:36
 */
#include "myslam/camera.h"

namespace myslam
{
    Camera::Camera(){}
    
    //有参构造，利用初始化列表
    /*
    Camera(
        float fx_, 
        float fy_, 
        float cx_, 
        float cy_, 
        float depth_scale_=0
    ):fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale){}
    */

    //相机坐标系的坐标变换,利用Eigen::Vector3d和Vector2d
    //三个坐标系：世界坐标系world、相机坐标系camera、成像坐标系pixel
    
    //Tcw*Pw = Pc
    Vector3d Camera::world2camera ( const Vector3d& p_w, const SE3& T_c_w ){
        return T_c_w*p_w;
    }
    //Twc*Pc = Pw
    Vector3d Camera::camera2world ( const Vector3d& p_c, const SE3& T_c_w ){
        return T_c_w.inverse() * p_c;
    }
    
    // u_x = f_x * Xc/Zc + c_x
    // u_y = f_y * Yc/Zc + c_y
    Vector2d Camera::camera2pixel ( const Vector3d& p_c ){
        return Vector2d( 
            fx_ * p_c(0,0) / p_c(2,0) + cx_,
            fy_ * p_c(1,0) / p_c(2,0) + cy_
        );
    }                 
    Vector3d Camera::pixel2camera ( const Vector2d& p_p, double depth=1 ){
        return Vector3d (
            ( p_p ( 0,0 )-cx_ ) *depth/fx_,
            ( p_p ( 1,0 )-cy_ ) *depth/fy_,
            depth
        );
    }
    
    // pixel2world = pixel2camera -> camera2world
    Vector3d Camera::pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth=1 ){
        return camera2pixel ( world2camera ( p_w, T_c_w ) );
    }
    // world2pixel = world2camera -> camera2pixel
    Vector2d Camera::world2pixel ( const Vector3d& p_w, const SE3& T_c_w ){
        return camera2world ( pixel2camera ( p_p, depth ), T_c_w );
    }


}