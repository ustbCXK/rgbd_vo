/*
 * @Descripttion: file content
 * @version: 
 * @Author: congsir
 * @Date: 2021-07-02 10:41:43
 * @LastEditors: Do not edit
 * @LastEditTime: 2021-07-02 10:45:51
 */
#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

//Eigen库
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

//Sophus库
#include <sophus/se3.h>
using Sophus::SE3;

//cv
#include <opencv2/core/core.hpp>
using cv::Mat;

//std基本库
#include<vector>
#include<list>
#include<memoty>
#include<string>
#include<iostream>
#include<set>
#include<unordered_map>
#include<map>
using namespace std;

#endif
