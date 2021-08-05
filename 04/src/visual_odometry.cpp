/*
 * @Descripttion: file content
 * @version: 
 * @Author: congsir
 * @Date: 2021-07-02 14:38:34
 * @LastEditors: Do not edit
 * @LastEditTime: 2021-07-05 11:28:17
 */

#include "myslam/config.h"
#include "myslam/visual_odometry.h"

#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/calib3d/calib3d.hpp>

#include<algorithm>
#include<boost/timer.hpp>

namespace myslam
{
VisualOdometry::VisualOdometry()//列表初始化
                        :state_ ( INITIALIZING ), 
                        ref_ ( nullptr ), 
                        curr_ ( nullptr ), 
                        map_ ( new Map ), 
                        num_lost_ ( 0 ), 
                        num_inliers_ ( 0 )
{
    num_of_features_    = Config::get<int> ( "number_of_features" );
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
   
    orb_ = cv::ORB::create ( num_of_features_, //特征点的个数
                             scale_factor_,     //尺度因子 orbslam是1.2
                             level_pyramid_ );  //金字塔的层数
}

VisualOdometry::~VisualOdometry()
{}

//这个函数为核心处理函数，添加的是新来的一帧，然后根据state_状态，选择是初始化还是计算T
bool VisualOdometry::addFrame( Frame::Ptr frame )
{
    //switch没啥问题
    switch (state_)
    {
    //第一帧肯定是初始化
    case INITIALIZING:
    {
        //先把状态改了，
        state_ = OK;
        //全部设置为第一帧
        curr_ = ref_ = frame;
        //插入地图中
        //map_->insertKeyFrame ( frame );
        // 提取特征点以及计算描述子，这里因为只有frame一帧，就不传参了
        extractKeyPoints();
        computeDescriptors();
        //之前增加关键帧需调用map类中的insertKeyFrame()函数，
        // 这里第一帧的话，就直接调用自身的addKeyFrame()函数添加进地图
        addKeyFrame();
        break;
    }
    //正常情况，匹配，然后调用poseEstimationPnP函数计算T
    case OK:
    {
        curr_ = frame;//ref_ 代表上一帧，只更新了curr_
        //说一下这一句，新的帧来了，先将其位姿赋值为参考帧的位姿，
        //因为考虑到匹配失败的情况下，这一帧就定义为丢失了，所以位姿就用参考帧的了。
        //如果一切正常，求得了当前帧的位姿，就进行赋值覆盖掉就好了。
        curr_->T_c_w_ = ref_->T_c_w_;
        extractKeyPoints(); //检测关键帧
        computeDescriptors();//计算当前帧
        featureMatching();  //匹配当前帧curr和参考帧ref的特征点
        poseEstimationPnP();    //位姿估计
        
        if ( checkEstimatedPose() == true ) // 判断估计是不是一个好的T，如果是就执行
        {
            //T比较好，旋转和平移都不是很大，计算当前的位姿
            // T_c_w = T_c_r*T_r_w
            curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;  
            //更新一下，将当前帧作为下一帧的参考帧 
            ref_ = curr_;
            //补全参考帧的depth数据，每次都要补全一下
            setRef3DPoints();
            num_lost_ = 0;
            //检测是否为关键帧,并插入
            if ( checkKeyFrame() == true ) // is a key-frame
            {
                addKeyFrame();
            }
        }
        else // 如果是坏的估计，匹配点太少，或者运动太大
        {
            //丢失帧数量加一
            num_lost_++;
            if ( num_lost_ > max_num_lost_ )
            {
                state_ = LOST;
            }
            return false;
        }
        break;
    }
    case LOST:
    {
        cout<<"vo has lost."<<endl;
        break;
    }
    }
    return true;
}

//内部处理函数，就特征匹配相关的
//提取特征点
void VisualOdometry::extractKeyPoints()
{
    //计算当前帧的特征点，存入keypoints_curr_中
    boost::timer timer;
    orb_->detect ( curr_->color_, keypoints_curr_ );
    cout<<"extract keypoints cost time: "<<timer.elapsed() <<endl;

}  

//计算特征描述子
void VisualOdometry::computeDescriptors()
{
    //计算所有特征点的描述子，存入descriptors_curr_中
     boost::timer timer;
    orb_->compute ( curr_->color_, keypoints_curr_, descriptors_curr_ );
    cout<<"descriptor computation cost time: "<<timer.elapsed() <<endl;
}  

//特征匹配
void VisualOdometry::featureMatching()
{
    //使用opencv暴力匹配
    vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING );//使用汉明距离
    //将当前帧和参考帧的匹配结果 存放到matches里
    matcher.match ( descriptors_ref_, descriptors_curr_, matches );

    //寻找最佳匹配，手动剔除
    //这里采用STL中的min_element和lambda表达式
    float min_dis = std::min_element (matches.begin(), matches.end(),[] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    //根据最小距离，对matches数组进行筛选，只有小于最小距离一定倍率的才有资格push进去
    //距离控制在一定范围内，提高可靠性
    feature_matches_.clear();
    for ( cv::DMatch& m : matches )
    {
        if ( m.distance < max<float>( min_dis*match_ratio_, 30.0 ) )
        {
            feature_matches_.emplace_back(m);
        }
    }
    cout <<"now there have "<<feature_matches_.size()<<"good matches!"<<endl;
}   

//利用PnP位姿估计  T_c_r_estimated_
void VisualOdometry::poseEstimationPnP()
{
    vector<cv::Point3f> pts3d;//3D点集合，来源于参考帧
    vector<cv::Point2f> pts2d;//2D点集合，来源于当前帧

    //当前帧的特征，以及参考帧的3D点
    for( cv::DMatch& m:feature_matches_ ){
        //pts_3d_ref_本来存储的就是3D点，直接索引取出
        pts3d.emplace_back( pts_3d_ref_[m.queryIdx] );
        //keypoints_curr_存储的是keypoint数组，所以定位索引后类型是keypoint
        //需要.pt获取对应的关键点的像素坐标
        pts2d.emplace_back( keypoints_curr_[ m.trainIdx ].pt );
    }
    
    //相机的内参矩阵
    Mat K = ( cv::Mat_<double>(3,3)<<
        ref_->camera_->fx_, 0, ref_->camera_->cx_,
        0, ref_->camera_->fy_, ref_->camera_->cy_,
        0,0,1
     );

     //结果：旋转向量，平移向量，内点数组
     Mat rvec, tvec, inliers;//矩阵形式
     //整个核心都是用cv::solvePnPRansac()求解两帧之间的位姿变化
     /*
     bool cv::solvePnPRansac 	( 	InputArray  	objectPoints,//3D点
            InputArray  	imagePoints,        //3D点
            InputArray  	cameraMatrix,       //内参矩阵K
            InputArray  	distCoeffs,         //距离系数
            OutputArray  	rvec,               
            OutputArray  	tvec,
            bool  	useExtrinsicGuess = false,
            int  	iterationsCount = 100,
            float  	reprojectionError = 8.0,
            double  	confidence = 0.99,
            OutputArray  	inliers = noArray(),
            int  	flags = SOLVEPNP_ITERATIVE 
	    ) 	
     */
     cv::solvePnPRansac(
         pts3d, //objectPoints
         pts2d, //imagePoints
         K,     //相机内参
         Mat(), //距离系数--0矩阵
         rvec,  //
         tvec, 
         false, 
         100, 
         4.0, 
         0.99, 
         inliers
     );
     //内点数量为内点的行数，所以等于rows
     num_inliers_  = inliers.rows;
     cout<<"pnp inliers: "<<num_inliers_<<endl;
     //根据旋转和平移构造出当前帧相对于参考帧的Trc，循环计算就能得到轨迹了！！！！
     T_c_r_estimated_ = SE3(
        SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)), 
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );
    //使用ba去优化位姿
    //都是老套路，初始化，主义由于更新所需要的的unique指针问题
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    
    //添加顶点，一帧有一个位姿，也就是只有一个顶点
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
        T_c_r_estimated_.rotation_matrix(), 
        T_c_r_estimated_.translation()
    ) );
    optimizer.addVertex ( pose );

    // edges
    //edges边有很多，每个特征点都对应一个重投影误差，也就是一条边
    for ( int i=0; i<inliers.rows; i++ )
    {
        int index = inliers.at<int>(i,0);
        // 3D -> 2D projection
        EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId(i);
        edge->setVertex(0, pose);
        edge->camera_ = curr_->camera_.get();
        edge->point_ = Vector3d( pts3d[index].x, pts3d[index].y, pts3d[index].z );
        edge->setMeasurement( Vector2d(pts2d[index].x, pts2d[index].y) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        optimizer.addEdge( edge );
    }
    
    //开始优化
    optimizer.initializeOptimization();
    //迭代次数
    optimizer.optimize(10);
    
    //将优化后的结果传出
    T_c_r_estimated_ = SE3 (
        pose->estimate().rotation(),
        pose->estimate().translation()
    );



}  


//关键帧的功能函数
//添加关键帧
void VisualOdometry::addKeyFrame()
{
    //增加关键帧函数多了一步在第一帧时，将其对应的地图点全部添加进地图中。
    if ( map_->keyframes_.empty() )
    {
        // first key-frame, add all 3d points into map
        for ( size_t i=0; i<keypoints_curr_.size(); i++ )
        {
            double d = curr_->findDepth ( keypoints_curr_[i] );
            if ( d < 0 ) 
                continue;
            Vector3d p_world = ref_->camera_->pixel2world (
                Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), curr_->T_c_w_, d
            );
            Vector3d n = p_world - ref_->getCamCenter();
            n.normalize();
            //上方求出构造地图点所需所有参数，3D点、模长、描述子、帧，然后构造一个地图点
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
            );
            //添加进地图
            map_->insertMapPoint( map_point );
        }
    }
    cout<<"adding a key-frame"<<endl;
    map_->insertKeyFrame ( curr_ );
}

//新增函数，增加地图中的点。局部地图类似于slidewindow一样，随时的增删地图中的点，来跟随运动
void VisualOdometry::addMapPoints()
{
    // add the new map points into map
    //创建一个bool型的数组matched，大小为当前keypoints数组大小，值全为false
    vector<bool> matched(keypoints_curr_.size(), false);
    //首先这个match_2dkp_index_是新来的当前帧跟地图匹配时，好的匹配到的关键点在keypoins数组中的索引
    //在这里将已经匹配的keypoint索引置为true
    for ( int index:match_2dkp_index_ )
        matched[index] = true;
    //遍历当前keypoints数组
    for ( int i=0; i<keypoints_curr_.size(); i++ )
    {
        //如果为true，说明在地图中找到了匹配，也就意味着地图中已经有这个点了。直接continue
        if ( matched[i] == true )   
            continue;
        //如果没有continue的话，说明这个点在地图中没有找到匹配，认定为新的点，
        //下一步就是找到depth数据，构造3D点，然后构造地图点，添加进地图即可。
        double d = ref_->findDepth ( keypoints_curr_[i] );
        if ( d<0 )  
            continue;
        Vector3d p_world = ref_->camera_->pixel2world (
            Vector2d ( keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y ), 
            curr_->T_c_w_, d
        );
        Vector3d n = p_world - ref_->getCamCenter();
        n.normalize();
        MapPoint::Ptr map_point = MapPoint::createMapPoint(
            p_world, n, descriptors_curr_.row(i).clone(), curr_.get()
        );
        map_->insertMapPoint( map_point );
    }
}

//新增函数：优化地图。主要是为了维护地图的规模。删除一些地图点，在点过少时增加地图点等操作。
void VisualOdometry::optimizeMap()
{
    // remove the hardly seen and no visible points
    //删除地图点，遍历地图中的地图点。并分几种情况进行删除。
    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
    {
        //如果点在当前帧都不可见了，说明跑的比价远了，删掉
        if ( !curr_->isInFrame(iter->second->pos_) )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        //定义匹配率，用匹配次数/可见次数，匹配率过低说明经常见但是没有几次匹配。应该是一些比较难识别的点，也就是出来的描述子比价奇葩。所以删掉
        float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
        if ( match_ratio < map_point_erase_ratio_ )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }

        double angle = getViewAngle( curr_, iter->second );
        if ( angle > M_PI/6. )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        //继续，可以根据一些其他条件自己添加要删除点的情况
        if ( iter->second->good_ == false )
        {
            // TODO try triangulate this map point 
        }
        iter++;
    }

    //下面说一些增加点的情况，首先当前帧去地图中匹配时，点少于100个了，
    // 一般情况是运动幅度过大了，跟之前的帧没多少交集了，所以增加一下。
    if ( match_2dkp_index_.size()<100 )
        addMapPoints();
    //如果点过多了，多于1000个，适当增加释放率，让地图处于释放点的趋势。
    if ( map_->map_points_.size() > 1000 )  
    {
        // TODO map is too large, remove some one 
        map_point_erase_ratio_ += 0.05;
    }
        //如果没有多于1000个，保持释放率在0.1，维持地图的体量为平衡态
    else 
        map_point_erase_ratio_ = 0.1;
    cout<<"map points: "<<map_->map_points_.size()<<endl;
}



//一个简单的位姿校验的函数，整体思路就是匹配点不能太少，运动不能太大
void VisualOdometry::checkEstimatedPose()
{
    //判断是否匹配点太少
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }

    //将变换矩阵取log操作得到变换向量。
    Sophus::Vector6d d = T_c_r_estimated_.log();
    //根据变换向量的模长来判断运动的大小。过大的话返回false
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm()<<endl;
        return false;
    }
    return true;
} 


//查看是否能构成关键帧，虽然没用到
void VisualOdometry::checkKeyFrame()
{
    //判断条件：很简单，T运动大就是关键帧，运动不大就不插入
    //虽然上述判断是否是不好的帧也用到了T的大小，但是量级不一样，
    //关键帧判断这里是跟0.1比，配置文件里有。判断错误那个是5 太大了
    Sophus::Vector6d d = T_c_r_estimated_.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
}       


};
}