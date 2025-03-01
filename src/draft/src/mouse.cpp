#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <cstdlib>

using namespace cv;
using namespace std;

const int width = 1280;
const int height = 720;
const string X_DIST_IMG_FILENAME = "120.png";
const string Y_DIST_IMG_FILENAME = "relative_y.png";

Mat x_distance_img, y_distance_img;
int x_offset_cm = 0;

// 鼠标点击回调函数：通过读取距离图像获取 x, y 距离
void onMouse(int event, int x, int y, int, void*)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        if (x_distance_img.empty() || y_distance_img.empty()) {
            cout << "距离图像未加载" << endl;
            return;
        }
        // 注意：Mat 的行列分别对应 y, x
        int x_pixel = static_cast<int>(x_distance_img.at<uchar>(y, x));
        int y_pixel = static_cast<int>(y_distance_img.at<uchar>(y, x));
        // 恢复实际距离（单位：m）
        double relative_x = (x_pixel - x_offset_cm) / 100.0;
        double relative_y = y_pixel / 100.0;
        cout << "点击像素(" << x << ", " << y << ") -> 相对坐标: X = " 
             << fixed << setprecision(3) << relative_x << " m, Y = " 
             << fixed << setprecision(3) << relative_y << " m" << endl;
    }
}

int main()
{
    // 加载距离图像（灰度图模式）
    x_distance_img = imread(X_DIST_IMG_FILENAME, IMREAD_GRAYSCALE);
    y_distance_img = imread(Y_DIST_IMG_FILENAME, IMREAD_GRAYSCALE);
    if (x_distance_img.empty() || y_distance_img.empty()) {
        cout << "读取距离图像失败，请确保图像已生成并与代码在同一目录下。" << endl;
        return -1;
    }

    // 从文件名中解析 x 图像对应的偏移量（单位：cm）
    try {
        // 假定文件名格式为 "数字.png"
        size_t pos = X_DIST_IMG_FILENAME.find('.');
        string numStr = X_DIST_IMG_FILENAME.substr(0, pos);
        x_offset_cm = stoi(numStr);
    } catch (...) {
        x_offset_cm = 0;
    }

    // 创建窗口并注册鼠标回调函数
    namedWindow("Video", WINDOW_AUTOSIZE);
    setMouseCallback("Video", onMouse, nullptr);

    // 打开摄像头视频流（默认摄像头ID为0）
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cout << "无法打开摄像头" << endl;
        return -1;
    }

    Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            cout << "无法获取视频帧" << endl;
            break;
        }
        // 缩放到预设尺寸，确保与校准参数匹配
        resize(frame, frame, Size(width, height));

        imshow("Video", frame);
        int key = waitKey(1);
        if (key == 27) {  // ESC 键退出
            break;
        }
    }

    cap.release();
    destroyAllWindows();
    return 0;
}