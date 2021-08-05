// -------------- test the visual odometry -------------
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp> 

#include "myslam/config.h"
#include "myslam/visual_odometry.h"


//写好了slam库，当然要试一下了
//example: ./run_vo path_to_config
int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }

    //查询配置文件
    myslam::Config::setParameterFile ( argv[1] );
    //构造VisualOdometry对象
    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );

    //数据集路径读取
    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;

    //针对于tum数据集，读取associate.txt文件 配准各项数据
    //如果没有，用python脚本对准
    ifstream fin ( dataset_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }

    //定义图片名字数组（string）和时间戳数组（double）
    //用于存放associate.txt文件中所示的时间戳对齐的RGB图像和depth图像
    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    //
    while ( !fin.eof() )
    {
        //associate.txt文件中的数据是string类型的，定义按顺序从fin中输入。
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );  //float时间戳
        depth_times.push_back ( atof ( depth_time.c_str() ) ); //float时间戳
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );   //rgb图片路径
        depth_files.push_back ( dataset_dir+"/"+depth_file );   //depth图片路径
        
        //good()返回是否读取到文件末尾，文件末尾处此函数会返回false。所以跳出
        if ( fin.good() == false )
            break;
    }

    //实例化一个相机对象
    myslam::Camera::Ptr camera ( new myslam::Camera );
    
    // visualization
    //利用opencv的Viz模块 
    //一、创建一个可视化窗口，构造参数为窗口的名称
    cv::viz::Viz3d vis("Visual Odometry");
    //二、创建坐标系部件，以Widget部件类型存在的，
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    
    //第三步、设置视角。这步是非必要步骤，进行设置有利于观察，
    //不设置也会有默认视角，就是可能比较别扭。而且开始后拖动鼠标，也可以改变观察视角。
    //构建三个3D点,这里其实就是构造makeCameraPose()函数需要的三个向量：
    //相机位置坐标、相机焦点坐标、相机y轴朝向
    //蓝色-Z，红色-X，绿色-Y
    cv::Point3d cam_pos( 0, -1.0, -1.0 ), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
    // cv::viz::makeCameraPose()构造相机在世界坐标系下位姿
    cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );
    //用setViewerPose()设置观看视角
    vis.setViewerPose( cam_pose );
    
    // 这里设置坐标系部件属性，然后添加到视图窗口上去
    // 首先利用setRenderingProperty()函数设置渲染属性，
    // 第一个参数是个枚举，对应要渲染的属性这里是线宽，后面是属性值
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    //用showWidget()函数将部件添加到窗口内
    vis.showWidget( "World", world_coor );
    vis.showWidget( "Camera", camera_coor );
    //至此，窗口中已经显示了全部需要显示的东西，就是两个坐标系：世界坐标系，相机坐标系。
    //世界坐标系就是写死不动的了，所以后面也没有再提起过世界坐标系。需要做的就是计算出各个帧的相机坐标系位置
    //后续的核心就是下面的for循环，在循环中，不断的给相机坐标系设置新的pose，然后达到动画的效果。


    //一共读取到几个rgb图像
    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
    //整个画面的快速刷新 呈现动态，由for循环控制
    for ( int i=0; i<rgb_files.size(); i++ )
    {
        //读取rgb和depth图像
        Mat color = cv::imread ( rgb_files[i] );
        Mat depth = cv::imread ( depth_files[i], -1 );
        //有一个为空，break
        if ( color.data==nullptr || depth.data==nullptr )
            break;
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];

        //利用boost库 查看实时性，就是计时
        boost::timer timer;
        vo->addFrame ( pFrame );
        //添加帧 需要时间
        cout<<"VO costs time: "<<timer.elapsed()<<endl;
        
        //LOST了 就跳出循环
        if ( vo->state_ == myslam::VisualOdometry::LOST )
            break;
        //Pw = Twc*Pc
        SE3 Tcw = pFrame->T_c_w_.inverse();
        
        // show the map and the camera pose 
        // 用Twc 构造Affine3d类型的pose 所需要的的旋转矩阵和平移矩阵
        cv::Affine3d M(
            cv::Affine3d::Mat3( 
                Tcw.rotation_matrix()(0,0), Tcw.rotation_matrix()(0,1), Tcw.rotation_matrix()(0,2),
                Tcw.rotation_matrix()(1,0), Tcw.rotation_matrix()(1,1), Tcw.rotation_matrix()(1,2),
                Tcw.rotation_matrix()(2,0), Tcw.rotation_matrix()(2,1), Tcw.rotation_matrix()(2,2)
            ), 
            cv::Affine3d::Vec3(
                Tcw.translation()(0,0), Tcw.translation()(1,0), Tcw.translation()(2,0)
            )
        );
        
        cv::imshow("image", color );
        cv::waitKey(1);
        vis.setWidgetPose( "Camera", M);
        vis.spinOnce(1, false);
    }

    return 0;
}
