#ifndef CONFIG_H
#define CONFIG_H    
/*
    Config 类负责参数文件的读取，并在程序任意地方都可随时提供参数的值。所以我们把
    Config 写成单件模式（Singleton）。它只有一个全局对象，当我们设置参数文件时，
    创建该对象并读取参数文件，随后就可以在任意地方访问参数值，最后在程序结束时自动销
    毁。
*/
#include "common_include.h"
namespace myslam
{
class Config
{
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config();   //    
public:
    ~Config(); // 析构函数

    //set a new config file 
    static void setParameterFile (const std::string& filename);

    template<typename T>
    static T get(const  std::string& key)
    {
        return T(Config::config_->file_[key]);
    }    
};
}
#endif //CONFIG_H

/*
    注解：1. 我们把构造函数声明为私有，防止这个类的对象在别处建立，它
    只能在 setParameterFile 时构造。实际构造的对象是 Config 的智
    能指针：static shared_ptr<Config>config_。用智能指针的原因是
    可以自动析构，省得我们再调一个别的函数来做析构。2. 在文件读取方面，
    我们使用 OpenCV 提供的 FileStorage 类。它可以读取一个 YAML文件，
    且可以访问其中任意一个字段。由于参数实质值可能为整数、浮点数或字符
    串，所以我们通过一个模板函数 get，来获得任意类型的参数值。
*/