//ELP-960P2CAM-V90-VC

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <functional>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/utility.hpp>

class DisparityCalculator : public rclcpp::Node {
public:
    DisparityCalculator() : Node("disparity_calculator") {
        // Subscritores para as imagens da esquerda e da direita
        left_sub.subscribe(this, "/camera/left/compressed");
        right_sub.subscribe(this, "/camera/right/compressed");

        // Sincronizador para garantir que processamos pares de imagens
        sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), left_sub, right_sub);
        sync->registerCallback(std::bind(&DisparityCalculator::image_callback, this, std::placeholders::_1, std::placeholders::_2));

        vis_mult = 2.8;
        wsize = 3;
        max_disp = 24;
        lambda = 80000.0;
        sigma = 1.8;
        
        if(max_disp%16 != 0)
            max_disp += 16-(max_disp%16);

        // Inicializar o algoritmo StereoSGBM
        left_matcher = cv::StereoSGBM::create(
            0,    // minDisparity
            max_disp,  // numDisparities
            wsize,   // blockSize
            100,  // P1
            1000, // P2
            64,    // disp12MaxDiff
            5);// preFilterCap
            left_matcher->setP1(max_disp * wsize * wsize);
            left_matcher->setP2(96 * wsize * wsize);
            left_matcher->setPreFilterCap(63);
            left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
            /*10,   // uniquenessRatio
            400,  // speckleWindowSize
            2,    // speckleRange
            cv::StereoSGBM::MODE_SGBM);
            */
        wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
        wls_filter->setLambda(lambda);
  	    wls_filter->setSigmaColor(sigma);
        right_matcher = cv::ximgproc::createRightMatcher(left_matcher);


        // Publicador para a nuvem de pontos
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);

        // Carregar parâmetros da câmara e calcular a matriz Q
        load_camera_parameters();

        // Publicar a transformação estática
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        publish_static_transform();
    }

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& left_msg, const sensor_msgs::msg::CompressedImage::ConstSharedPtr& right_msg) {
        try {
            // Descomprimir as imagens
            cv::Mat left_img_color  = cv::imdecode(cv::Mat(left_msg->data), cv::IMREAD_COLOR);
            cv::Mat right_img_color = cv::imdecode(cv::Mat(right_msg->data), cv::IMREAD_COLOR);

            if (left_img_color.empty() || right_img_color.empty()) {
                RCLCPP_WARN(this->get_logger(), "Falha ao decodificar uma ou ambas as imagens!");
                return;
            }
           
            // Converter para escala de cinzentos
            cv::Mat left_gray, right_gray;
            cv::cvtColor(left_img_color, left_gray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(right_img_color, right_gray, cv::COLOR_BGR2GRAY);

            // Calcular o mapa de disparidade
            left_matcher->compute(left_gray, right_gray, left_disp);
            right_matcher->compute(right_gray, left_gray, right_disp);
            wls_filter->filter(left_disp, left_gray, filtered_disp, right_disp);
            conf_map = wls_filter->getConfidenceMap();
  	        ROI = wls_filter->getROI();

            //cv::Mat filtered_disp_vis;
            //cv::ximgproc::getDisparityVis(filtered_disp, filtered_disp_vis, vis_mult);
            // Gerar e publicar a nuvem de pontos
            cv::medianBlur(filtered_disp, filtered_disp, 5);
            cv::Mat points_3d;
            cv::reprojectImageTo3D(filtered_disp, points_3d, Q_, true);
            publish_point_cloud(points_3d, left_img_color, left_msg->header);

            // Normalizar o mapa de disparidade para visualização (0-255)
            cv::Mat disparity_normalized;
            cv::normalize(filtered_disp, disparity_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);

            // Mostrar as imagens originais e o mapa de disparidade
            cv::imshow("Imagem Esquerda", left_img_color);
            cv::imshow("Imagem Direita", right_img_color);
            cv::imshow("Mapa de Disparidade", disparity_normalized);
            cv::waitKey(1);
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao processar imagens: %s", e.what());
        }
    }

    void load_camera_parameters() {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("elp");
        std::string left_yaml_path = package_share_directory + "/config/left.yaml";
        std::string right_yaml_path = package_share_directory + "/config/right.yaml";

        YAML::Node left_cam_info = YAML::LoadFile(left_yaml_path);
        
        cv::Mat K_left = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 9; ++i) K_left.at<double>(i / 3, i % 3) = left_cam_info["camera_matrix"]["data"][i].as<double>();

        double fx = K_left.at<double>(0, 0);
        double cx = K_left.at<double>(0, 2);
        double cy = K_left.at<double>(1, 2);

        double baseline = 0.075; // Assumir 7.5cm. AJUSTAR ESTE VALOR! medido de centro a centro das lentes mas sem rigor (fita métrica)
        
        Q_ = cv::Mat::zeros(4, 4, CV_64F);
        Q_.at<double>(0, 0) = 1.0;
        Q_.at<double>(0, 3) = -cx;
        Q_.at<double>(1, 1) = 1.0;
        Q_.at<double>(1, 3) = -cy;
        Q_.at<double>(2, 3) = fx;
        Q_.at<double>(3, 2) = -1.0 / baseline;
    }

    void publish_point_cloud(const cv::Mat& points_3d, const cv::Mat& color_image, const std_msgs::msg::Header& header) {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.header = pcl_conversions::toPCL(header);
        cloud.is_dense = false;
        cloud.width = points_3d.cols;
        cloud.height = points_3d.rows;

        for (int y = 0; y < points_3d.rows; ++y) {
            for (int x = 0; x < points_3d.cols; ++x) {
                cv::Vec3f point = points_3d.at<cv::Vec3f>(y, x);
                pcl::PointXYZRGB pcl_point;
                
                pcl_point.x = point[0];
                pcl_point.y = point[1];
                pcl_point.z = point[2];
                
                cv::Vec3b color = color_image.at<cv::Vec3b>(y, x);
                pcl_point.r = color[2];
                pcl_point.g = color[1];
                pcl_point.b = color[0];
                
                cloud.points.push_back(pcl_point);
            }
        }

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(cloud, output_msg);
        point_cloud_pub_->publish(output_msg);
    }

    void publish_static_transform() {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "camera_frame";

        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_static_broadcaster_->sendTransform(t);
    }

    // Definir a política de sincronização
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage> SyncPolicy;
    
    // Membros da classe
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> left_sub;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> right_sub;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;
    cv::Ptr<cv::StereoSGBM> left_matcher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    cv::Mat Q_, left_disp, right_disp, filtered_disp, conf_map;
    cv::Rect ROI ;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
    cv::Ptr<cv::StereoMatcher> right_matcher;
    double vis_mult;
    int wsize, max_disp;
    double lambda, sigma;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisparityCalculator>());
    rclcpp::shutdown();
    return 0;
}



