#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/mat.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

class CameraPublisher : public rclcpp::Node {
public:
    image_transport::Publisher publisher_left;
    image_transport::Publisher publisher_right;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_right_pub_;
    rclcpp::TimerBase::SharedPtr timer;
    cv::VideoCapture cap;

    sensor_msgs::msg::CameraInfo camera_info_left;
    sensor_msgs::msg::CameraInfo camera_info_right;

    CameraPublisher() : Node("camera_publisher") {
        // Declarar parâmetros com valores padrão
        this->declare_parameter<int>("frame_width", 640);
        this->declare_parameter<int>("frame_height", 480);
        this->declare_parameter<int>("rate", 10);
        this->declare_parameter<int>("dev_video", 0);

        // Obter valores dos parâmetros
        int width = this->get_parameter("frame_width").as_int();
        int height = this->get_parameter("frame_height").as_int();
        int rate = this->get_parameter("rate").as_int();
        int dev_video = this->get_parameter("dev_video").as_int();

        // Criar publicador
        publisher_left = image_transport::create_publisher(this, "/camera/left");
        publisher_right = image_transport::create_publisher(this, "/camera/right");
        camera_info_left_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/left/camera_info", 10);
        camera_info_right_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/right/camera_info", 10);
        // Abrir a câmera (0 para a câmera padrão)
        cap.open(dev_video, cv::CAP_V4L2);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao abrir a câmera!");
            rclcpp::shutdown();
        }
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

        // Criar timer para capturar e publicar imagens
        timer = this->create_wall_timer(
            std::chrono::milliseconds(1000 / rate),
            std::bind(&CameraPublisher::timer_callback, this)
        );

        camera_info_left = loadCameraInfo("/home/jpalves/dev_ws/src/elp/config/left.yaml");
        camera_info_right = loadCameraInfo("/home/jpalves/dev_ws/src/elp/config/right.yaml");
    }

    void publish_image(const cv::Mat& frame, image_transport::Publisher publisher, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub, sensor_msgs::msg::CameraInfo& info) {
        if (!frame.empty()) {
            std_msgs::msg::Header hdr;
            hdr.stamp = this->now();
            hdr.frame_id = "camera_frame";
            auto msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
            publisher.publish(msg);
            cv::waitKey(1);
            info.header = msg->header;
            info_pub->publish(info);
        }
    }

    void timer_callback() {
        cv::Mat img, esquerda, direita;
        this->cap >> img;  // Capturar frame da câmera
        if (img.empty()) return;
        int y = img.rows; 
        int x = img.cols;
        esquerda = img(cv::Rect(0, 0, x/2, y));
        direita = img(cv::Rect(x/2, 0, x/2, y));
        //cv::imshow("Esquerda", esquerda);
        //cv::imshow("Direita", direita);
        //cv::waitKey(1);
        this->publish_image(esquerda, this->publisher_left, this->camera_info_left_pub_, this->camera_info_left);
        this->publish_image(direita, this->publisher_right, this->camera_info_right_pub_, this->camera_info_right);
    }

    sensor_msgs::msg::CameraInfo loadCameraInfo(const std::string& yaml_path) {
        sensor_msgs::msg::CameraInfo info;
        YAML::Node config = YAML::LoadFile(yaml_path);
        info.width = config["image_width"].as<int>();
        info.height = config["image_height"].as<int>();
        info.distortion_model = config["distortion_model"].as<std::string>();
        // Camera matrix (K)
        auto k_data = config["camera_matrix"]["data"];
        for (size_t i = 0; i < info.k.size(); ++i) info.k[i] = k_data[i].as<double>();
        // Distortion coefficients (D)
        auto d_data = config["distortion_coefficients"]["data"];
        info.d.resize(d_data.size());
        for (size_t i = 0; i < d_data.size(); ++i) info.d[i] = d_data[i].as<double>();
        // Rectification matrix (R)
        auto r_data = config["rectification_matrix"]["data"];
        for (size_t i = 0; i < info.r.size(); ++i) info.r[i] = r_data[i].as<double>();
        // Projection matrix (P)
        auto p_data = config["projection_matrix"]["data"];
        for (size_t i = 0; i < info.p.size(); ++i) info.p[i] = p_data[i].as<double>();
        return info;
    }
};

// Certificar que a classe termina corretamente
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}

