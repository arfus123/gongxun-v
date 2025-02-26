#include "parameters.h"
#include <iostream>

std::string camInput;
std::string camOutputX;
std::string camOutputY;

double fx;
double fy;
double cx;
double cy;
double k1;
double k2;
double p1;
double p2;
double k3;

int width;
int height;
int fps;
int RGB;

void readParameters(const std::string &config_file)
{
    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cerr << "Failed to open config file: " << config_file << std::endl;
        return;
    }

    fs["Camera1.input"] >> camInput;
    fs["Camera1.outputx"] >> camOutputX;
    fs["Camera1.outputy"] >> camOutputY;

    // 读取摄像机内参
    fs["Camera1.fx"] >> fx;
    fs["Camera1.fy"] >> fy;
    fs["Camera1.cx"] >> cx;
    fs["Camera1.cy"] >> cy;
    
    // 读取畸变系数
    fs["Camera1.k1"] >> k1;
    fs["Camera1.k2"] >> k2;
    fs["Camera1.p1"] >> p1;
    fs["Camera1.p2"] >> p2;
    fs["Camera1.k3"] >> k3;
    
    // 读取分辨率、帧率和色彩顺序
    fs["Camera1.width"] >> width;
    fs["Camera1.height"] >> height;
    fs["Camera1.fps"] >> fps;
    fs["Camera1.RGB"] >> RGB;
    
    fs.release();
    
    std::cout << "Loaded config:" << std::endl;
    std::cout << "fx: " << fx << ", fy: " << fy << std::endl;
    std::cout << "cx: " << cx << ", cy: " << cy << std::endl;
    std::cout << "Distortion: k1=" << k1 << ", k2=" << k2 
              << ", p1=" << p1 << ", p2=" << p2 << ", k3=" << k3 << std::endl;
    std::cout << "Resolution: " << width << "x" << height << std::endl;
    std::cout << "FPS: " << fps << ", RGB order: " << RGB << std::endl;
}