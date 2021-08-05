/*
 * @Descripttion: file content
 * @version: 
 * @Author: congsir
 * @Date: 2021-07-02 12:02:06
 * @LastEditors: Do not edit
 * @LastEditTime: 2021-07-02 13:25:50
 */


#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam
{
class MapPoint;
class Frame
{
public://成员变量
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long id_;  //当前帧的id
    double time_stamp_;  //时间戳
    SE3 T_c_w_;         //当前帧的位姿// transform from world to camera
    Camera::Ptr camera_;    //当前帧对应的相机
    Mat color_, depth_;     //当前帧的图像，rgb图和深度图

public://成员函数s
    
    //构造函数
    Frame();
    //TODO：可以改成初始化列表形式
    //带参构造
    Frame(
        long id,
        double time_stamp=0,
        SE3 T_c_w = SE3(),// transform from world to camera
        Camera::Ptr camera = nullptr,
        Mat color = Mat(),
        Mat depth = Mat()
    );
    //析构函数
    ~Frame();//可以不写
    
    
    //创建关键帧函数
    static Frame::Ptr createFrame();

    //从深度图得到深度
    /*keypoint默认构造函数：
    CV_WRAP KeyPoint() : pt(0,0), size(0), angle(-1), response(0), octave(0), class_id(-1) {}
        pt(x,y):关键点的点坐标；
        size():该关键点邻域直径大小；
        angle:角度，表示关键点的方向，值为[零,三百六十)，负值表示不使用。
        response:响应强度
    */
    double findDepth( const cv::KeyPoint& kp );

    //求得图像中心
    Vector3d getCamCenter() const;

    //查看当前点是否在帧内
    bool isInFrame( const Vector3d& pt_world );
};
}