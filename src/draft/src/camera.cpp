#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include "parameters.h"

using namespace cv;
using namespace std;

int main() {
    readParameters("/home/firefly/gongxun-v/config/fv100.yaml");
    // 相机内参矩阵（示例值，需替换为实际值）
    Mat K = (Mat_<double>(3, 3) << fx, 0, cx,
                                    0, fy, cy,
                                    0, 0, 1);
    // 畸变系数（示例值，需替换为实际值）
    Mat distCoeffs = (Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
    
    // 相机参数设置
    double theta = setAngle * CV_PI / 180.0; // 俯仰角，度转换为弧度
    double H = setHeight; // 相机离地面的高度（米）

    // 构建绕X轴的旋转矩阵（俯仰角）
    Mat R = Mat::eye(3, 3, CV_64F);
    R.at<double>(1, 1) = cos(theta);
    R.at<double>(1, 2) = sin(theta);
    R.at<double>(2, 1) = -sin(theta);
    R.at<double>(2, 2) = cos(theta);
    
    // 创建查找表，保存每个像素对应的地面坐标(X,Y)
    Mat lookupTableX(height, width, CV_64F);
    Mat lookupTableY(height, width, CV_64F);
    
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            // 将像素坐标转换为归一化坐标（去畸变）
            vector<Point2f> src = {Point2f(u, v)};
            vector<Point2f> dst;
            undistortPoints(src, dst, K, distCoeffs, noArray(), K);
            
            double x = dst[0].x;
            double y = dst[0].y;
            
            // 构造相机坐标系中的方向向量
            Mat vec_cam = (Mat_<double>(3, 1) << x, y, 1.0);
            
            // 转换到世界坐标系中的方向向量
            Mat vec_world = R * vec_cam;
            
            double vz = vec_world.at<double>(2);
            if (fabs(vz) < 1e-6) { // 避免除零
                lookupTableX.at<double>(v, u) = 0;
                lookupTableY.at<double>(v, u) = 0;
                continue;
            }
            
            // 计算交点参数
            double t = -H / vz;
            double X = t * vec_world.at<double>(0);
            double Y = t * vec_world.at<double>(1);
            
            lookupTableX.at<double>(v, u) = X;
            lookupTableY.at<double>(v, u) = Y;
        }
    }
    
    cv::Mat dst, dst_8u, lookupTableX_8u, lookupTableY_8u;
    cvtColor(lookupTableX, lookupTableX_8u, cv::COLOR_GRAY2BGR);
    cv::equalizeHist(lookupTableX_8u, dst);
    dst.convertTo(dst_8u, CV_8U, 255.0); 

    // 显示均衡化后的图像
    cv::imshow("Equalized Image", dst_8u);
    // std::cout << lookupTableX << std::endl;
    // std::cout << lookupTableY << std::endl;

    // 以下代码生成三通道图像，并将前两通道各自以灰度图形式显示
    // 归一化查找表数据到 [0, 255]，便于以8位图像显示，注意不同通道可能归一化范围不同
    Mat normX, normY;
    normalize(lookupTableX, normX, 0, 255, NORM_MINMAX, CV_8U);
    normalize(lookupTableY, normY, 0, 255, NORM_MINMAX, CV_8U);
    
    // 调试代码，检查 normX 和 normY 是否为空
    if (normX.empty() || normY.empty()) {
        cerr << "Error: normX or normY is empty!" << endl;
        return -1;
    }

    // 用第三通道填充零
    Mat zeroChannel = Mat::zeros(normX.size(), normX.type());

    // 合并三个通道：第一通道保存x轴坐标，第二通道保存y轴坐标，第三通道全0
    vector<Mat> channels = {normX, normY, zeroChannel};
    Mat combined;
    merge(channels, combined);
    
    // 保存合并后的三通道图像到当前文件夹下
    imwrite("results.png", combined);
    
    // 分别显示x轴和y轴坐标的灰度图
    imshow("X Coordinate (Grayscale)", normX);
    imshow("Y Coordinate (Grayscale)", normY);
    waitKey(0);
    
    return 0;
}