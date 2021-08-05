/*
 * @Descripttion: file content
 * @version: 
 * @Author: congsir
 * @Date: 2021-07-02 14:38:29
 * @LastEditors: Do not edit
 * @LastEditTime: 2021-07-05 11:12:39
 */
#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace myslam
{
class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState{
        INITIALIZING = -1,  //初始化
        OK=0,               //正常
        LOST                //丢失
    }

    //存放两两帧VO所用到的参考帧和当前帧，还有VO状态及整个地图
    VOState state_;     //VO状态
    Map::Ptr map_;      //整个地图
    Frame::Ptr ref_;    //参考帧
    Frame::Ptr curr_;   //当前帧

    //存放两帧匹配需要的keypoints，descriptors，matches
    cv::Ptr<cv::ORB> orb_;  //orb检测和计算
    //PnP嘛 就是3D-2D
    //vector<cv::Point3f> pts_3d_ref_;    //参考帧的3d点的数组
    vector<cv::KeyPoint> keypoints_curr_;   //当前帧的特征点的数组
    Mat descriptors_curr_;  //当前帧的描述子
    //Mat descriptors_ref_;   //参考帧的描述子
    //vector<cv::DMatch> feature_matches_;//特征点的匹配关系
    
    cv::FlannBasedMatcher   matcher_flann_;
    vector<MapPoint::Ptr>   match_3dpts_;
    vector<int>             match_2dkp_index_; 
    //存放匹配结果T，还有表示结果好坏的内点数和丢失书
    SE3 T_c_w_estimated_;   //估计得到的当前帧到参考帧的位姿
    int num_inliers_;        // ICP中内特征点的个数
    int num_lost_;           // lost的次数，如果超过一定值就直接down掉

    //参数
    int num_of_features_;   //特征点的个数
    double scale_factor_;   //图像金字塔的尺度
    int level_pyramid_;     //金字塔level的数量
    float match_ratio_;     //好点的比率
    int max_num_lost_;      //最大连续丢失次数
    int min_inliers_;       //最小的内点数

    //用于判定是否为关键帧的标准，
    //说白了就是规定一定幅度的旋转和平移，大于这个幅度就归为关键帧
    double key_frame_min_rot;   // 两个关键帧最小R
    double key_frame_min_trans; // 两个关键帧最小t

public://函数
    VisualOdometry();
    ~VisualOdometry();

    //这个函数为核心处理函数，将帧添加进来，然后处理。
    bool addFrame( Frame::Ptr frame );

protected:
    //内部处理函数，就特征匹配相关的
    void extractKeyPoints();    //提取特征点
    void computeDescriptors();  //计算特征描述子
    void featureMatching();     //特征匹配
    void poseEstimationPnP();   //利用PnP位姿估计
    void setRef3DPoints();      //构建参考帧的3D点

    //关键帧的功能函数
    void addKeyFrame();         //添加关键帧
    //增加地图点函数
    void addMapPoints();
    void checkEstimatedPose();  //验证估计的位姿
    void checkKeyFrame();       //验证关键帧
    double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );

};

}