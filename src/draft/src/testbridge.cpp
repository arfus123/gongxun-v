#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include "chrono"
#include "yolov8.hpp"

using namespace std;

// 定义一个小球结构体来存储检测到的小球信息
struct Ball {
    std::string color;
    float x;
    float y;
};
std::vector<Ball> balls;

std::vector<Object> objs;

extern const char* class_names[];
const std::string engine_file_path = "/home/firefly/Downloads/model_zoo-master/gongxun/1201/gx1201.trt";
auto yolov8 = new YOLOv8(engine_file_path);

const string X_DIST_IMG_FILENAME = "/home/firefly/gongxun-v/src/draft/config/120.png";
const string Y_DIST_IMG_FILENAME = "/home/firefly/gongxun-v/src/draft/config/relative_y.png";

cv::Mat        x_distance_img, y_distance_img;
cv::Size       im_size(640, 640);
const int      num_labels  = 4;
const int      topk        = 100;
const float    score_thres = 0.30f;
const float    iou_thres   = 0.65f;
int            x_offset_cm = 0;

class BallDetector
{
public:
    BallDetector()
    {
        cout << "Begin class...\n" << endl;
        image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &BallDetector::imageCallback, this);
        obj_info_pub_ = nh_.advertise<std_msgs::String>("/detect/obj_info", 1);
        for(i=0;i<16;i++) FPS[i]=0.0;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            // 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV 图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            std::vector<Object> objs;
            yolov8->CopyFromMat(cv_ptr->image, im_size);
            Tbegin = std::chrono::steady_clock::now();
            yolov8->Infer();
            yolov8->PostProcess(objs, score_thres, iou_thres, topk, num_labels);
            yolov8->DrawObjects(cv_ptr->image, objs);   
            Tend = std::chrono::steady_clock::now();
            GetObjPositions(balls, objs);
            // 将小球信息转换为 JSON 格式
            std::string json_msg = ballsToJson(balls);

            // 发布 JSON 消息
            std_msgs::String obj_info_msg;
            obj_info_msg.data = json_msg;
            obj_info_pub_.publish(obj_info_msg);
            balls.clear();

            // 显示图像（可选）
            f = std::chrono::duration_cast <std::chrono::milliseconds> (Tend - Tbegin).count();
            if(f>0.0) FPS[((Fcnt++)&0x0F)]=1000.0/f;
            for(f=0.0, i=0;i<16;i++){ f+=FPS[i]; }
            putText(cv_ptr->image, cv::format("FPS %0.2f", f/16),cv::Point(10,20),cv::FONT_HERSHEY_SIMPLEX,0.6, cv::Scalar(0, 0, 255));
            cv::imshow("Camera Image", cv_ptr->image);
            char esc = cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Publisher obj_info_pub_;
    std::string message;
    cv::Mat  image;
    std::chrono::steady_clock::time_point Tbegin, Tend;
    float    f;
    float    FPS[16];
    int      i, Fcnt=0;
    
    void GetObjPositions(std::vector<Ball>& balls, const std::vector<Object>& objs)
    {
        if (x_distance_img.empty() || y_distance_img.empty()) {
            cout << "距离图像未加载" << endl;
            return;
        }
        for (auto& obj : objs) {
            Ball ball;
            ball.color = class_names[obj.label];
            int x_pixel = static_cast<int>(x_distance_img.at<uchar>(obj.rect.y, obj.rect.x));
            int y_pixel = static_cast<int>(y_distance_img.at<uchar>(obj.rect.y+obj.rect.height/2.0, obj.rect.x));
            ball.x = (x_pixel - x_offset_cm) / 100.0;
            ball.y = y_pixel / 100.0;
            balls.push_back(ball);
        }
    }

    // std::vector<Ball> detectBalls(const cv::Mat& image)
    // {
    //     std::vector<Ball> balls;
    //     // 示例：假设我们检测到四个固定位置和颜色的小球
    //     balls.push_back({"red", 1.1, 0.2});
    //     balls.push_back({"blue", 2.3, 1.4});
    //     balls.push_back({"black", 0.5, 0.9});
    //     balls.push_back({"yellow", 3.2, 2.1});

    //     return balls;
    // }

    std::string ballsToJson(const std::vector<Ball>& balls)
    {
        nlohmann::json json_data;
        json_data["nun"] = balls.size();
        json_data["time"] = static_cast<int>(ros::Time::now().toSec());

        for (const auto& ball : balls)
        {
            nlohmann::json ball_info;
            ball_info["color"] = ball.color;
            std::ostringstream oss_x, oss_y;
            oss_x << std::fixed << std::setprecision(2) << ball.x;
            oss_y << std::fixed << std::setprecision(2) << ball.y;
            ball_info["x"] = oss_x.str();
            ball_info["y"] = oss_x.str();
            json_data["balls"].push_back(ball_info);
        }

        return json_data.dump(); // 转换为字符串
    }
};

int main(int argc, char** argv)
{
    cout << "Reading distance images..." << endl;
    x_distance_img = cv::imread(X_DIST_IMG_FILENAME, cv::IMREAD_GRAYSCALE);
    y_distance_img = cv::imread(Y_DIST_IMG_FILENAME, cv::IMREAD_GRAYSCALE);
    if (x_distance_img.empty() || y_distance_img.empty()) {
        cout << "Read failed" << endl;
        return -1;
    }
    try {
        // 文件名格式为 "数字.png"
        size_t pos = X_DIST_IMG_FILENAME.find('.');
        string numStr = X_DIST_IMG_FILENAME.substr(0, pos);
        x_offset_cm = stoi(numStr);
    } catch (...) {
        x_offset_cm = 0;
    }

    cout << "Set CUDA...\n" << endl;
    cudaSetDevice(0);
    cout << "Loading TensorRT model " << engine_file_path << endl;
    cout << "\nWait a second...." << std::flush;
    cout << "\rLoading the pipe... " << string(10, ' ')<< "\n\r" ;
    cout << endl;
    yolov8->MakePipe(true);

    ros::init(argc, argv, "ball_detector");
    BallDetector bd;

    ros::spin();
    return 0;
}
