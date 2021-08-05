/*
 * @Descripttion: file content
 * @version: 
 * @Author: congsir
 * @Date: 2021-07-02 13:13:50
 * @LastEditors: Do not edit
 * @LastEditTime: 2021-07-02 13:29:43
 */

#include "myslam/map.h"

namespace myslam
{
    Map(){}
    void Map::insertKeyFrame(Frame::Ptr frame)
    {
        cout<<"  Key frame size that befort insert "<<keyframes_.size()<<endl;
        //地图里没有就插入当前帧，有了就更新一下帧
        if( keyframes_.find(frame->id_) == keyframes_.end() )
        {
            keyframes.insert( make_pair(frame->id_, frame) );
        }
        else
        {
            keyframes_[ frame->id_ ] = frame;
        }
    }

    void Map::insertMapPoint(MapPoint::Ptr map_point)
    {
        if ( map_points_.find(map_point->id_) == map_points_.end() )
        {
            map_points_.insert( make_pair(map_point->id_, map_point) );
        }
        else 
        {
            map_points_[map_point->id_] = map_point;
        }
    }
}