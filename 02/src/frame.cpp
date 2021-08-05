/*
 * @Descripttion: file content
 * @version: 
 * @Author: congsir
 * @Date: 2021-07-02 12:02:18
 * @LastEditors: Do not edit
 * @LastEditTime: 2021-07-02 13:10:33
 */
#include "myslam/frame.h"

namespace myslam
{
    //构造函数
    Frame::Frame()
    :id(-1),time_stamp_(-1), camera_(nullptr)
    {

    }


    //TODO：可以改成初始化列表形式
    //带参构造
    Frame::Frame(
        long id,
        double time_stamp,
        SE3 T_c_w,
        Camera::Ptr camera,
        Mat color,
        Mat depth
    ): id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth)
    {
    }


    //析构函数
    Frame::~Frame(){}//暂时可以不写
    
    
    //创建关键帧函数
    Frame::Ptr Frame::createFrame()
    {
        static long factory_id =0;
        //直接返回一个Frame类指针，方便直接使用
        return Frame::Ptr( new Frame(factory_id++) );
    }

    //寻找给定点的深度
    /*keypoint默认构造函数：
    CV_WRAP KeyPoint() : pt(0,0), size(0), angle(-1), response(0), octave(0), class_id(-1) {}
        pt(x,y):关键点的点坐标；
        size():该关键点邻域直径大小；
        angle:角度，表示关键点的方向，值为[零,三百六十)，负值表示不使用。
        response:响应强度
    */
    double Frame::findDepth( const cv::KeyPoint& kp )
    {
        //cvRound():返回跟参数最接近的整数值，即四舍五入；
        int x = cvRound(kp.pt.x);
        int y = cvRound(kp.pt.y);
        //根据x和y在深度图中进行索引，并取得depth值
        ushort d = depth_.ptr<ushort>(y)[x];//y是某一行，x是某一列，规定的形式，直接使用
        if(d){
            return double (d)/camera_->depth_scale_;  //d/相机内参的深度因子，因为归一化
        } 
        else{//d=0,取附近的值，技巧来了，不枉我刷题啊
            int dx[4] = {-1,0,1,0},int dy[4] = {0,-1,0,1};
            for( int i=0;i<4;++i ){
                d = depth_.ptr<ushort>( y+dy[i] )[x+dx[i]];
                if(d!=0) return double (d)/camera_->depth_scale_;
            }
        }
        return -1.0;//如果周围也是0，就返回-1
    }

    //求得图像中心
    Vector3d Frame::getCamCenter() const
    {
        //第一帧是世界坐标系，所以Twc就是位姿，
        //取得位姿Twc 是相机到世界坐标系的转换，其中的translation()就是平移向量，
        //也就是相机光心在世界坐标系的坐标了
        return T_c_w_.inverse().translation();
    }

    //查看当前点是否在帧内
    bool Frame::isInFrame( const Vector3d& pt_world )
    {
        //给定一个点，此时为世界坐标系，将世界坐标系转到相机坐标系下的坐标：
        // p_cam为相机坐标系下的坐标
        Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
        if( p_cam(2,0) < 0 )return false;//p_cam==(X,Y,Z) Z=0肯定就不在范围了
        Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
        //在帧内的要求是 xy值 都大于0并且小于color图的行列
        return pixel(0,0)>0
            && pixel(0,0)<color_.cols//小于列
            && pixel(1,0)>0
            && pixel(1,0)<color_.rows;//小于行
    }
}