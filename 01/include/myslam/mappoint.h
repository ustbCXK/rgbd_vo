/*
 * @Descripttion: file content
 * @version: 
 * @Author: congsir
 * @Date: 2021-07-02 12:57:00
 * @LastEditors: Do not edit
 * @LastEditTime: 2021-07-02 13:08:10
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
    Vector3d pos_;      //三维空间的坐标
    Vector3d norm_;     //正则化的坐标？
    Mat descriptor_;    //特征描述子
    int observed_times_;//被特征匹配算法观测的次数
    int correct_times_; //在位姿估计中作为inliners的次数
public:
    MapPoint();
    MapPoint(
        long id,
        Vector3d position,
        Vector3d norm
    );

    //创建地图点函数
    static MapPoint::Ptr createMapPoint();
}
}

#endif  //MAPPOINT_H