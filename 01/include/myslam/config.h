/*
 * @Descripttion: file content
 * @version: 
 * @Author: congsir
 * @Date: 2021-07-02 13:30:16
 * @LastEditors: Do not edit
 * @LastEditTime: 2021-07-02 13:51:03
 */
/*
    各项参数文件的读取，并且在程序的任何地方都可以随时提取参数
    为满足需求 写成单例模式
    将构造函数设置为私有的，只有自己类成员才能调用构造函数
*/

#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/common_include.h" 

namespace myslam
{
class Config
{
private:
    static std::shared_ptr<Config> config_; 
    
    cv::FileStorage file_; //可以读写一个yaml文件且访问随意字段

    //构造函数设置为私有，防止类对象在别的地方实例化，
    //这样只能在setParameterFile调用时构造，实际构造对象是智能指针
    //智能指针可以自动析构
    Config () {}

    
public:
    ~Config();//关闭文件

    //添加一个新的配置文件
    static void setParameterFile( const std::string& filename ); 

    //为方便读取各种格式的数据，浮点、int等等用模板函数吧
    template< typename T >
    static T get( const std::string& key )
    {
        return T( Config::config_->file_[key] );
    }
    

};
}
#endif //CONFIG_H