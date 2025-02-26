#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

int main() {
    // 相机内参矩阵（示例值，需替换为实际值）
    Mat K = (Mat_<double>(3, 3) << 800, 0, 320,
                                    0, 800, 240,
                                    0, 0, 1;
    // 畸变系数（示例值，需替换为实际值）
    Mat distCoeffs = (Mat_<double>(5, 1) << 0.1, -0.01, 0.001, 0.001, 0);
    
    // 相机参数设置
    double theta = 30.0 * CV_PI / 180.0; // 俯仰角，30度转换为弧度
    double H = 1.5; // 相机离地面的高度（米）
    
    // 图像尺寸（示例值，需替换为实际值）
    int width = 640;
    int height = 480;
    
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
    
    // 示例：输出中心像素的坐标
    int centerU = width / 2;
    int centerV = height / 2;
    cout << "中心像素(" << centerU << ", " << centerV << ") 的地面坐标: ("
         << lookupTableX.at<double>(centerV, centerU) << ", "
         << lookupTableY.at<double>(centerV, centerU) << ")" << endl;
    
    return 0;
}