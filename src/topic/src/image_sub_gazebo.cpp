#include "global.hpp"
#include "yolo.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "Mat2image.hpp"
// #include "ros2_interfaces/msg/coord.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <ctime>

#define USE_CUDA true
using namespace Eigen;
using namespace std;
using namespace cv;

char cir_coord[50];
char H_coord[50];
char cir_last_frame[50];
char H_last_frame[50];
double target_x = 317;
double target_y = 338;
int flag_servo = 0;
int cir_count = 0;
int H_count = 0;
int model_flag = 0;
string model_path = "models/best_H.onnx";
string model_path_circle = "models/best_circle.onnx";
Matrix3d K; // 内参矩阵
Vector2d D; // 畸变矩阵

class imageSub : public rclcpp::Node
{
public:
    imageSub() : Node("webcam_sub") {
        isFirstFrame = true;
        
        std::cout << "OpenCV version: " << CV_VERSION << std::endl;
        readmodel();

        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_topic", 
            10, 
            std::bind(&imageSub::image_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("detection2d_array", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&imageSub::timer_callback, this)
        );

    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    VideoCapture cap;
    Yolo test;
    dnn::Net net1, net2;
    vector<Output> result;
    bool isFirstFrame;
    VideoWriter video;
    rclcpp::Time last_message_time_;
    bool message_received_;

    void readmodel() {
        if (test.readModel(net1, model_path, USE_CUDA))
            cout << "read net1 ok!" << endl;
        else
            cout << "read onnx1 model failed!";
        if (test.readModel(net2, model_path_circle, USE_CUDA))
            cout << "read net2 ok!" << endl;
        else
            cout << "read onnx2 model failed!";
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        message_received_ = true;

        RCLCPP_INFO(this->get_logger(), "Receiving video frames");

        if (msg == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Received null image message");
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (cv_ptr->image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Converted image is empty");
            return;
        }

        cv::Mat src = cv_ptr->image;

        // 检查图像尺寸是否有效
        if (src.rows <= 0 || src.cols <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid image dimensions: rows=%d, cols=%d", src.rows, src.cols);
            return;
        }


        //cv::imshow("frame", src);
        Mat img = handle_image(src, test);

        namedWindow("frame", WINDOW_NORMAL);
        cv::circle(img, cv::Point(target_x,target_y), 20, cv::Scalar(0,255,255), 2, cv::LINE_AA);// 黄色
		// resizeWindow("frame", 780, 500);
		// moveWindow("frame", 10000, 10000);
        cv::imshow("frame", img);
        waitKey(1);

		drive_servo(cir_coord, target_x, target_y, 0.5); //时间
        if (strcmp(cir_coord, cir_last_frame) == 0) { 
            cir_count++;
            if (cir_count >= 30)
            {
                strcpy(cir_coord, "0");
                cir_count = 0;
            }
        }
        if (strcmp(H_coord, H_last_frame) == 0) { 
            H_count++;
            if (H_count >= 150)
            {
                strcpy(H_coord, "0");
                H_count = 0;
            }
        }
        
        strcpy(cir_last_frame, cir_coord);
        strcpy(H_last_frame, H_coord);


        // if (isFirstFrame)
        // {
        //     time_t now = time(0);
        //     tm *ltm = localtime(&now);
        //     char timestamp[20];
        //     sprintf(timestamp, "%04d-%02d-%02d_%02d:%02d:%02d", 1900 + ltm->tm_year, 1 + ltm->tm_mon, ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
        //     string videoFilename = "/home/linhao/Videos/" + string(timestamp) + ".avi";
        //     video.open(videoFilename, VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, Size(img.cols, img.rows));
        //     if (!video.isOpened())
        //     {
        //         cerr << "Error: Failed to create video file: " << videoFilename << endl;
        //     }
        //     isFirstFrame = false;
        // }
        // video.write(img);
    }


    Mat handle_image(Mat frame, Yolo& test)
    {
        cv::Mat K = (cv::Mat_<double>(3, 3) << 514.0045, 0, 321.6074,
                                               0, 514.6655, 260.0872,
                                               0, 0, 1);
        cv::Mat D = (cv::Mat_<double>(4, 1) << 0.1631, -0.2023, 0, 0);
        vector<Scalar> color;

        srand(time(0));
        for (int i = 0; i < 80; i++) {
            int b = rand() % 256;
            int g = rand() % 256;
            int r = rand() % 256;
            color.push_back(Scalar(b, g, r));
        }

        Mat cir_img, H_img, img;
        undistort(frame, img, K, D, K);
        if (test.Detect(img, net1, result, 1)) {
            img = test.drawPred(img, result, color, 1);
        }
        if (test.Detect(img, net2, result, 0)) {
            test.target(img, result, 0);
            img = test.drawPred(img, result, color, 0);
        }
        return img;
    }

    void timer_callback() {
        // 发布 Detection2DArray 消息
        vision_msgs::msg::Detection2DArray detection_array;
        detection_array.header.stamp = this->get_clock()->now();
        detection_array.header.frame_id = "camera_frame";
        
        // 如果有圆形目标坐标
        float cir_x, cir_y;
        int servo;
        if (sscanf(cir_coord, "%f,%f,%d", &cir_x, &cir_y, &servo) >= 2 && cir_x != 0 && cir_y != 0) {
            vision_msgs::msg::Detection2D detection;
            detection.header = detection_array.header;
            
            // 设置边界框
            detection.bbox.center.position.x = cir_x;
            detection.bbox.center.position.y = cir_y;
            detection.bbox.center.theta = 0.0;
            detection.bbox.size_x = 50.0;  // 假设的边界框宽度
            detection.bbox.size_y = 50.0;  // 假设的边界框高度
            
            // 设置检测结果
            vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
            hypothesis.hypothesis.class_id = "circle";  // 圆形目标的类别ID
            hypothesis.hypothesis.score = 0.9;     // 假设的置信度
            detection.results.push_back(hypothesis);
            
            detection_array.detections.push_back(detection);
        }
        
        // 如果有H形目标坐标
        float h_x, h_y;
        if (sscanf(H_coord, "%f, %f", &h_x, &h_y) == 2 && h_x != 0 && h_y != 0) {
            vision_msgs::msg::Detection2D detection;
            detection.header = detection_array.header;
            
            // 设置边界框
            detection.bbox.center.position.x = h_x;
            detection.bbox.center.position.y = h_y;
            detection.bbox.center.theta = 0.0;
            detection.bbox.size_x = 50.0;  // 假设的边界框宽度
            detection.bbox.size_y = 50.0;  // 假设的边界框高度
            
            // 设置检测结果
            vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
            hypothesis.hypothesis.class_id = "h";  // H形目标的类别ID
            hypothesis.hypothesis.score = 0.9;     // 假设的置信度
            detection.results.push_back(hypothesis);
            
            detection_array.detections.push_back(detection);
        }
        
        // 发布 Detection2DArray 消息
        publisher_->publish(detection_array);
        // RCLCPP_INFO(this->get_logger(), "Published Detection2DArray with %zu detections", detection_array.detections.size());
    }

    void drive_servo(char *str, double target_x, double target_y, double duration) {
        float x, y, r;
        float rmax = 20;
        static bool start_timing = false;
        if (sscanf(str, "%f,%f", &x, &y) == 2) {
            r = calculateDistance(x, y, target_x, target_y);

            if (r <= rmax)
                if (start_timing == false) {
                    start_timing = true;
                    last_message_time_ = this->get_clock()->now();
                    RCLCPP_INFO(this->get_logger(), "begin to timing");
                } else {
                    rclcpp::Time current_time = this->get_clock()->now();
                    if ((current_time - last_message_time_).seconds() >= duration) {
                        RCLCPP_INFO(this->get_logger(), "flag_servo has set to 1");
                        flag_servo = 1;
                    }
                }
            else {
                start_timing = false;
                flag_servo = 0;
            }
        }
    }

    double calculateDistance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double distance = std::sqrt(dx * dx + dy * dy);
        return distance;
    }
};



int main(int argc, char ** argv)
{
    cir_coord[0] = '0';
    H_coord[0] = '0';
    rclcpp::init(argc, argv);
    auto node = std::make_shared<imageSub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}

