#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

class CubeDetector : public rclcpp::Node{

        public:
                CubeDetector() : Node("cube_detector"){

                        RCLCPP_INFO(this->get_logger(), "CubeDetector is launching...");

                        // Seuils HSV (OpenCV : H dans [0, 180], S et V dans [0, 255]).
                        // Le rouge est a cheval sur H = 0, d'ou deux plages.
                        red_hue_max_ = this->declare_parameter<int>("red_hue_max", 10);
                        red_hue_min_ = this->declare_parameter<int>("red_hue_min", 170);
                        // Vert pur : H = 60
                        green_hue_min_ = this->declare_parameter<int>("green_hue_min", 45);
                        green_hue_max_ = this->declare_parameter<int>("green_hue_max", 75);
                        // Bleu pur : H = 120
                        blue_hue_min_ = this->declare_parameter<int>("blue_hue_min", 105);
                        blue_hue_max_ = this->declare_parameter<int>("blue_hue_max", 135);

                        saturation_min_ = this->declare_parameter<int>("saturation_min", 120);
                        value_min_ = this->declare_parameter<int>("value_min", 70);
                        min_area_ = this->declare_parameter<double>("min_area", 500.0);

                        // Pose de la camera dans le monde (cf. capteur camera_deportee dans panda_world.sdf)
                        camera_x_ = this->declare_parameter<double>("camera_x", 2.07);
                        camera_y_ = this->declare_parameter<double>("camera_y", 0.0);
                        camera_z_ = this->declare_parameter<double>("camera_z", 2.16);
                        camera_roll_ = this->declare_parameter<double>("camera_roll", 0.0);
                        camera_pitch_ = this->declare_parameter<double>("camera_pitch", 0.5176);
                        camera_yaw_ = this->declare_parameter<double>("camera_yaw", 3.1416);
                        camera_hfov_ = this->declare_parameter<double>("camera_hfov", 1.1519);
                        cube_size_ = this->declare_parameter<double>("cube_size", 0.06);
                        // Erosion (px de l'image couleur) de la silhouette avant de lire la profondeur,
                        // pour ne pas prendre les pixels du bord qui tombent sur un autre objet
                        depth_erode_px_ = this->declare_parameter<int>("depth_erode_px", 15);
                        // Centile des hauteurs mesurees pris comme dessus du cube (robuste aux pixels parasites)
                        top_percentile_ = this->declare_parameter<double>("top_percentile", 0.9);

                        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                                "/deported_camera/image", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
                                std::bind(&CubeDetector::image_callback, this, std::placeholders::_1)
                        );

                        // Profondeur le long de l'axe optique, meme pose et meme champ de vision
                        // que la camera couleur mais en plus basse resolution
                        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                                "/deported_camera/depth_image", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
                                [this](const sensor_msgs::msg::Image::ConstSharedPtr msg){ last_depth_ = msg; }
                        );

                        // Position du centre du cube dans le repere monde (frame "map")
                        red_cube_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
                                "/cube_detector/red_cube", 10
                        );
                        green_cube_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
                                "/cube_detector/green_cube", 10
                        );
                        blue_cube_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
                                "/cube_detector/blue_cube", 10
                        );

                        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                                "/cube_detector/debug_image", 10
                        );

                        RCLCPP_INFO(this->get_logger(), "CubeDetector successfuly launched");
                }

        private:

                struct Detection{
                        std::string name;
                        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher;
                        bool found = false;
                        std::vector<cv::Point> contour;
                        cv::Mat mask;           // silhouette remplie du cube
                        double u = 0.0;
                        double v = 0.0;
                        Eigen::Vector3d position;
                };

                // Pixel (u, v) -> direction du rayon dans le monde, normalisee pour que sa
                // composante le long de l'axe optique vaille 1 (point = camera + profondeur * rayon)
                Eigen::Vector3d pixel_to_ray(double u, double v, int width, int height){

                        // Modele pinhole : pixels carres, centre optique au milieu de l'image
                        double f = width / (2.0 * std::tan(camera_hfov_ / 2.0));
                        double cx = width / 2.0;
                        double cy = height / 2.0;

                        // Repere capteur Gazebo : x devant, y a gauche, z en haut
                        Eigen::Vector3d ray_camera(1.0, -(u - cx) / f, -(v - cy) / f);

                        Eigen::Matrix3d rotation =
                                (Eigen::AngleAxisd(camera_yaw_, Eigen::Vector3d::UnitZ())
                               * Eigen::AngleAxisd(camera_pitch_, Eigen::Vector3d::UnitY())
                               * Eigen::AngleAxisd(camera_roll_, Eigen::Vector3d::UnitX())).toRotationMatrix();

                        return rotation * ray_camera;
                }

                // Pixel (u, v) -> point du monde : on coupe le rayon avec le plan horizontal z = plane_z
                Eigen::Vector3d pixel_to_world(double u, double v, int width, int height, double plane_z){

                        Eigen::Vector3d camera_position(camera_x_, camera_y_, camera_z_);
                        Eigen::Vector3d ray_world = pixel_to_ray(u, v, width, height);

                        double t = (plane_z - camera_z_) / ray_world.z();
                        return camera_position + t * ray_world;
                }

                // Garde la plus grande zone du masque (le cube), apres nettoyage du bruit
                void detect(cv::Mat& mask, const cv::Mat& kernel, Detection& detection){

                        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
                        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

                        std::vector<std::vector<cv::Point>> contours;
                        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                        int best = -1;
                        double best_area = min_area_;
                        for (size_t i = 0; i < contours.size(); ++i){
                                double area = cv::contourArea(contours[i]);
                                if (area > best_area){
                                        best_area = area;
                                        best = static_cast<int>(i);
                                }
                        }

                        if (best < 0){
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s cube not found", detection.name.c_str());
                                return;
                        }

                        detection.found = true;
                        detection.contour = contours[best];
                        detection.mask = cv::Mat::zeros(mask.size(), CV_8UC1);
                        cv::drawContours(detection.mask, contours, best, cv::Scalar(255), cv::FILLED);

                        cv::Moments m = cv::moments(detection.contour);
                        detection.u = m.m10 / m.m00;
                        detection.v = m.m01 / m.m00;
                }

                // Hauteur du centre du cube a partir de la profondeur : on reconstruit en 3D les points
                // vus a l'interieur de la silhouette, le plus haut d'entre eux est le dessus du cube.
                // (Sur un cube du milieu d'une pile, le dessus est cache mais l'arete haute de la face
                // avant est a la meme hauteur.)
                bool compute_center_z(const Detection& detection, const cv::Mat& depth, double& center_z){

                        cv::Mat mask;
                        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                cv::Size(2 * depth_erode_px_ + 1, 2 * depth_erode_px_ + 1));
                        cv::erode(detection.mask, mask, kernel);

                        double scale_x = static_cast<double>(mask.cols) / depth.cols;
                        double scale_y = static_cast<double>(mask.rows) / depth.rows;

                        std::vector<double> heights;
                        for (int dv = 0; dv < depth.rows; ++dv){
                                int v = static_cast<int>((dv + 0.5) * scale_y);
                                for (int du = 0; du < depth.cols; ++du){
                                        int u = static_cast<int>((du + 0.5) * scale_x);
                                        if (!mask.at<uchar>(v, u)) continue;

                                        float d = depth.at<float>(dv, du);
                                        if (!std::isfinite(d)) continue;

                                        heights.push_back(camera_z_ + d * pixel_to_ray(du, dv, depth.cols, depth.rows).z());
                                }
                        }

                        if (heights.empty()) return false;

                        size_t k = static_cast<size_t>(top_percentile_ * (heights.size() - 1));
                        std::nth_element(heights.begin(), heights.begin() + k, heights.end());
                        center_z = heights[k] - cube_size_ / 2.0;
                        return true;
                }

                void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg){

                        if (!last_depth_){
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for depth image");
                                return;
                        }

                        cv_bridge::CvImageConstPtr cv_image, cv_depth;
                        try{
                                cv_image = cv_bridge::toCvShare(msg, "bgr8");
                                cv_depth = cv_bridge::toCvShare(last_depth_, "32FC1");
                        }
                        catch (const cv_bridge::Exception& e){
                                RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
                                return;
                        }

                        cv::Mat hsv;
                        cv::cvtColor(cv_image->image, hsv, cv::COLOR_BGR2HSV);

                        // Supprime le bruit gaussien de la camera puis rebouche les trous
                        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));

                        Detection red, green, blue;
                        red.name = "Red";
                        red.publisher = red_cube_pub_;
                        green.name = "Green";
                        green.publisher = green_cube_pub_;
                        blue.name = "Blue";
                        blue.publisher = blue_cube_pub_;

                        // ---------- Cube rouge ----------

                        cv::Mat red_mask_low, red_mask_high, red_mask;
                        cv::inRange(hsv, cv::Scalar(0, saturation_min_, value_min_),
                                    cv::Scalar(red_hue_max_, 255, 255), red_mask_low);
                        cv::inRange(hsv, cv::Scalar(red_hue_min_, saturation_min_, value_min_),
                                    cv::Scalar(180, 255, 255), red_mask_high);
                        red_mask = red_mask_low | red_mask_high;
                        detect(red_mask, kernel, red);

                        // ---------- Cube vert ----------

                        cv::Mat green_mask;
                        cv::inRange(hsv, cv::Scalar(green_hue_min_, saturation_min_, value_min_),
                                    cv::Scalar(green_hue_max_, 255, 255), green_mask);
                        detect(green_mask, kernel, green);

                        // ---------- Cube bleu ----------

                        cv::Mat blue_mask;
                        cv::inRange(hsv, cv::Scalar(blue_hue_min_, saturation_min_, value_min_),
                                    cv::Scalar(blue_hue_max_, 255, 255), blue_mask);
                        detect(blue_mask, kernel, blue);

                        // ---------- Position 3D et publication ----------

                        cv::Mat debug = cv_image->image.clone();

                        for (Detection* d : {&red, &green, &blue}){
                                if (!d->found) continue;

                                double center_z;
                                if (!compute_center_z(*d, cv_depth->image, center_z)){
                                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s cube: no valid depth", d->name.c_str());
                                        continue;
                                }
                                // Le centre de la silhouette est sur le rayon du centre du cube : on le coupe
                                // a la hauteur mesuree
                                d->position = pixel_to_world(d->u, d->v, msg->width, msg->height, center_z);

                                geometry_msgs::msg::PointStamped point;
                                point.header.stamp = msg->header.stamp;
                                point.header.frame_id = "map";
                                point.point.x = d->position.x();
                                point.point.y = d->position.y();
                                point.point.z = d->position.z();
                                d->publisher->publish(point);

                                std::vector<std::vector<cv::Point>> contours = {d->contour};
                                cv::drawContours(debug, contours, 0, cv::Scalar(0, 255, 255), 5);
                                cv::circle(debug, cv::Point(d->u, d->v), 15, cv::Scalar(255, 255, 255), -1);
                                cv::putText(debug, d->name + " z=" + std::to_string(static_cast<int>(std::round(d->position.z() * 1000))) + "mm", cv::Point(d->u + 30, d->v),
                                            cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 6);
                        }

                        debug_image_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", debug).toImageMsg());
                }

                int red_hue_max_;
                int red_hue_min_;
                int green_hue_min_;
                int green_hue_max_;
                int blue_hue_min_;
                int blue_hue_max_;
                int saturation_min_;
                int value_min_;
                double min_area_;
                double camera_x_;
                double camera_y_;
                double camera_z_;
                double camera_roll_;
                double camera_pitch_;
                double camera_yaw_;
                double camera_hfov_;
                double cube_size_;
                int depth_erode_px_;
                double top_percentile_;

                rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
                rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
                sensor_msgs::msg::Image::ConstSharedPtr last_depth_;
                rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr red_cube_pub_;
                rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr green_cube_pub_;
                rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr blue_cube_pub_;
                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
};

int main(int argc, char** argv){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<CubeDetector>());
        rclcpp::shutdown();
        return 0;
}
