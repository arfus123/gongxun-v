#include <opencv2/opencv.hpp>
#include <iostream>
#include "parameters.h"

int main()
{
    // 先加载并打印配置文件中的参数
    readParameters("/home/firefly/gongxun-v/config/gf100.yaml");

    std::cout << "\n----- 打印加载的参数 -----" << std::endl;
    std::cout << "Input: " << camInput << std::endl;
    std::cout << "OutputX: " << camOutputX << std::endl;
    std::cout << "OutputY: " << camOutputY << std::endl;
    std::cout << "fx: " << fx << ", fy: " << fy << std::endl;
    std::cout << "cx: " << cx << ", cy: " << cy << std::endl;
    std::cout << "Distortion: k1=" << k1 
              << ", k2=" << k2 
              << ", p1=" << p1 
              << ", p2=" << p2 
              << ", k3=" << k3 << std::endl;
    std::cout << "Resolution: " << width << " x " << height << std::endl;
    std::cout << "FPS: " << fps << ", RGB: " << RGB << std::endl;
    std::cout << "---------------------------" << std::endl << std::endl;
    
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "无法打开摄像头。" << std::endl;
        return -1;
    }
    
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);
    cv::Mat frame;
    
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "无法读取视频帧。" << std::endl;
            break;
        }
        cv::imshow("Camera", frame);
        if (cv::waitKey(30) >= 0) {
            break;
        }
    }
    
    cap.release();
    cv::destroyAllWindows();
    return 0;
}