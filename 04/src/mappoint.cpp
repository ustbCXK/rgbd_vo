/*
 * @Descripttion: file content
 * @version: 
 * @Author: congsir
 * @Date: 2021-07-02 12:57:09
 * @LastEditors: Do not edit
 * @LastEditTime: 2021-07-05 11:08:39
 */

#include "myslam/common_include.h"
#include "myslam/mappoint.h"

namespace myslam
{
    //默认构造
    MapPoint::MapPoint():id_(-1), pos_(Vector3d(0,0,0)), norm_(Vector3d(0,0,0)), observed_times_(0), correct_times_(0)
    {}

    //构造函数，将观察帧 push_back进去
    MapPoint::MapPoint ( long unsigned int id, const Vector3d& position, const Vector3d& norm, Frame* frame, const Mat& descriptor )
: id_(id), pos_(position), norm_(norm), good_(true), visible_times_(1), matched_times_(1), descriptor_(descriptor)
{
    observed_frames_.push_back(frame);
}

    //创建地图点函数，直接在累加上id 构造一个就行，返回定义的地图点mapPoint指针
    MapPoint::Ptr MapPoint::createMapPoint()
    {
        static long factory_id =0;
        return MapPoint::Ptr( 
            new MapPoint( factory_id++, Vector3d(0,0,0), Vector3d(0,0,0) )
        );
    }
    MapPoint::Ptr MapPoint::createMapPoint ( 
    const Vector3d& pos_world, 
    const Vector3d& norm, 
    const Mat& descriptor, 
    Frame* frame )
{
    return MapPoint::Ptr( 
        new MapPoint( factory_id_++, pos_world, norm, frame, descriptor )
    );
}
unsigned long MapPoint::factory_id_ = 0;
}