/*
 * @Descripttion: file content
 * @version: 
 * @Author: congsir
 * @Date: 2021-07-02 13:13:45
 * @LastEditors: Do not edit
 * @LastEditTime: 2021-07-02 13:22:03
 */
/*
    Map管理所有mapPoints和所有的frame，
        功能函数包括添加新路标、删除原路标点等，
        VO过程主要是调用这些函数操作mapPoints

    由于存储了各个关键帧和路标点，
    所以需要随机访问并随机插入和删除，
    所以这里用散列(Hashmap)来存储，也就是键值对。
*/
#ifndef MAP_H
#define MAP_H
#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam
{
class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    unordered_map<unsigned long, MapPoint::Ptr> map_points_;//所有地图点
    unordered_map<unsigned long, Frame::Ptr> keyframes_;    //所有关键帧
public:        
    Map();
    void insertKeyFrame(Frame::Ptr frame);
    void insertMapPoint(MapPoint::Ptr map_point);

};
}
#endif //MAP_H