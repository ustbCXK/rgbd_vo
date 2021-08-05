/*
 * @Descripttion: file content
 * @version: 
 * @Author: congsir
 * @Date: 2021-07-02 12:57:00
 * @LastEditors: Do not edit
 * @LastEditTime: 2021-07-05 11:05:44
 */
/*
    mappoint代表地图点，我们的任务是估计他在世界坐标系中的坐标
并且将当前帧提取的特征点和地图点进行匹配，来估计相机的运动，
因此其中还需要存储某特征点对应的描述子，还要记录某个点被观测的次数和被匹配的次数
    1、地图id
    2、世界坐标系中坐标
    3、直接可观测的关系
    4、描述子
    5、被特征匹配算法观测的次数
    6、在位姿估计中作为inliners的粗疏
*/

#ifndef MAPPOINT_H
#define MAPPOINT_H
#include "myslam/common_include.h"

namespace myslam
{
class Frame;
class MapPoint
{
public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long id;   //特征点的id
    static unsigned long factory_id_;
    bool        good_; 
    Vector3d pos_;      //三维空间的坐标
    Vector3d norm_;     //正则化的坐标？
    Mat descriptor_;    //特征描述子
    list<Frame*>    observed_frames_;//记录观察到该地图点的帧
    
    int observed_times_;//被特征匹配算法观测的次数
    int correct_times_; //在位姿估计中作为inliners的粗疏

public:
    MapPoint();
    MapPoint(
        unsigned long id,
        const Vector3d& position, 
        const Vector3d& norm, 
        Frame* frame=nullptr,   //记录观察到该地图点的帧
        const Mat& descriptor=Mat() //地图点描述子
    );

    //取得地图点3维坐标的功能函数
    inline cv::Point3f getPositionCV() const {
        return cv::Point3f( pos_(0,0), pos_(1,0), pos_(2,0) );
    }


    //创建地图点函数
    static MapPoint::Ptr createMapPoint();
    static MapPoint::Ptr createMapPoint( 
        const Vector3d& pos_world, 
        const Vector3d& norm_,
        const Mat& descriptor,
        Frame* frame );
}
}

#endif  //MAPPOINT_H