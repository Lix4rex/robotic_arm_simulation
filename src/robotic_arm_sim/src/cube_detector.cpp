#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

class CubeDetector : public rclcpp::Node{

        public:
                CubeDetector() : Node("cube_detector"){

                        RCLCPP_INFO(this->get_logger(), "CubeDetector is launching...");

                        // Couleurs a detecter : nom et plage de teinte par defaut
                        // (OpenCV : H dans [0, 180], S et V dans [0, 255]).
                        // Si hue_min > hue_max, la plage est a cheval sur H = 0 (cas du rouge).
                        // Chaque couleur donne les parametres <nom>_hue_min / <nom>_hue_max
                        // et le topic /cube_detector/<nom>_cube
                        cubes_ = {
                                {"red",    170, 10},   // Rouge pur : H = 0
                                {"green",  45,  75},   // Vert pur : H = 60
                                {"blue",   105, 135},  // Bleu pur : H = 120
                                {"yellow", 20,  35},   // Jaune pur : H = 30
                                {"cyan", 80,  100},   // Jaune pur : H = 90
                                {"magenta", 140,  160},   // Jaune pur : H = 150
                        };

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
                        // Erosion (px de l'image couleur) avant de reconstruire les points servant a l'orientation :
                        // plus faible que depth_erode_px pour garder les aretes, les intrus sont filtres en 3D
                        yaw_erode_px_ = this->declare_parameter<int>("yaw_erode_px", 4);
                        // Nombre minimal de points 3D pour estimer l'orientation
                        yaw_min_points_ = this->declare_parameter<int>("yaw_min_points", 30);

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

                        for (CubeColor& cube : cubes_){
                                cube.hue_min = this->declare_parameter<int>(cube.name + "_hue_min", cube.hue_min);
                                cube.hue_max = this->declare_parameter<int>(cube.name + "_hue_max", cube.hue_max);
                                // Pose du centre du cube dans le repere monde (frame "map") : position + lacet
                                // (rotation autour de z, ramene dans [-pi/4, pi/4[ par symetrie du cube)
                                cube.publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                                        "/cube_detector/" + cube.name + "_cube", 10
                                );
                        }

                        debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                                "/cube_detector/debug_image", 10
                        );

                        RCLCPP_INFO(this->get_logger(), "CubeDetector successfuly launched");
                }

        private:

                struct CubeColor{
                        std::string name;
                        int hue_min;
                        int hue_max;
                        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher = nullptr;
                };

                struct Detection{
                        std::string name;
                        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
                        bool found = false;
                        std::vector<cv::Point> contour;
                        cv::Mat mask;           // silhouette remplie du cube
                        double u = 0.0;
                        double v = 0.0;
                        Eigen::Vector3d position;
                        double yaw = 0.0;
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

                // Pixels de l'image HSV dans la plage de teinte de la couleur
                cv::Mat color_mask(const cv::Mat& hsv, const CubeColor& cube){

                        cv::Mat mask;
                        if (cube.hue_min <= cube.hue_max){
                                cv::inRange(hsv, cv::Scalar(cube.hue_min, saturation_min_, value_min_),
                                            cv::Scalar(cube.hue_max, 255, 255), mask);
                                return mask;
                        }

                        // Plage a cheval sur H = 0 : [hue_min, 180] U [0, hue_max]
                        cv::Mat mask_low, mask_high;
                        cv::inRange(hsv, cv::Scalar(0, saturation_min_, value_min_),
                                    cv::Scalar(cube.hue_max, 255, 255), mask_low);
                        cv::inRange(hsv, cv::Scalar(cube.hue_min, saturation_min_, value_min_),
                                    cv::Scalar(180, 255, 255), mask_high);
                        mask = mask_low | mask_high;
                        return mask;
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

                // Points du monde vus par la camera de profondeur a l'interieur de la silhouette,
                // erodee de erode_px pixels (image couleur)
                std::vector<Eigen::Vector3d> reconstruct_points(const cv::Mat& silhouette, const cv::Mat& depth, int erode_px){

                        cv::Mat mask = silhouette;
                        if (erode_px > 0){
                                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                        cv::Size(2 * erode_px + 1, 2 * erode_px + 1));
                                cv::erode(silhouette, mask, kernel);
                        }

                        double scale_x = static_cast<double>(mask.cols) / depth.cols;
                        double scale_y = static_cast<double>(mask.rows) / depth.rows;
                        Eigen::Vector3d camera_position(camera_x_, camera_y_, camera_z_);

                        std::vector<Eigen::Vector3d> points;
                        for (int dv = 0; dv < depth.rows; ++dv){
                                int v = static_cast<int>((dv + 0.5) * scale_y);
                                for (int du = 0; du < depth.cols; ++du){
                                        int u = static_cast<int>((du + 0.5) * scale_x);
                                        if (!mask.at<uchar>(v, u)) continue;

                                        float d = depth.at<float>(dv, du);
                                        if (!std::isfinite(d)) continue;

                                        points.push_back(camera_position + d * pixel_to_ray(du, dv, depth.cols, depth.rows));
                                }
                        }
                        return points;
                }

                // Hauteur du centre du cube a partir de la profondeur : on reconstruit en 3D les points
                // vus a l'interieur de la silhouette, le plus haut d'entre eux est le dessus du cube.
                // (Sur un cube du milieu d'une pile, le dessus est cache mais l'arete haute de la face
                // avant est a la meme hauteur.)
                bool compute_center_z(const Detection& detection, const cv::Mat& depth, double& center_z){

                        std::vector<double> heights;
                        for (const Eigen::Vector3d& p : reconstruct_points(detection.mask, depth, depth_erode_px_)){
                                heights.push_back(p.z());
                        }

                        if (heights.empty()) return false;

                        size_t k = static_cast<size_t>(top_percentile_ * (heights.size() - 1));
                        std::nth_element(heights.begin(), heights.begin() + k, heights.end());
                        center_z = heights[k] - cube_size_ / 2.0;
                        return true;
                }

                // Lacet du cube : les points vus (dessus et faces laterales) projetes sur le plan horizontal
                // tombent dans l'empreinte carree du cube, dont on prend le rectangle englobant minimal.
                // Resultat dans [-pi/4, pi/4[ (le cube est invariant par rotation de pi/2)
                bool compute_yaw(const Detection& detection, const cv::Mat& depth, double& yaw){

                        // On ne garde que les points appartenant au cube : entre sa base (exclue, pour ne pas
                        // prendre la table ou le cube du dessous) et son dessus, et pres de son axe vertical
                        double z_min = detection.position.z() - cube_size_ / 2.0 + 0.005;
                        double z_max = detection.position.z() + cube_size_ / 2.0 + 0.01;
                        double max_radius = 0.75 * cube_size_;   // demi-diagonale ~ 0.71 * cote

                        std::vector<cv::Point2f> footprint;
                        for (const Eigen::Vector3d& p : reconstruct_points(detection.mask, depth, yaw_erode_px_)){
                                if (p.z() < z_min || p.z() > z_max) continue;
                                if (std::hypot(p.x() - detection.position.x(), p.y() - detection.position.y()) > max_radius) continue;
                                footprint.emplace_back(p.x(), p.y());
                        }

                        if (static_cast<int>(footprint.size()) < yaw_min_points_) return false;

                        cv::Point2f corners[4];
                        cv::minAreaRect(footprint).points(corners);

                        // Direction d'un cote du rectangle, dans le repere monde (x, y)
                        double angle = std::atan2(corners[1].y - corners[0].y, corners[1].x - corners[0].x);
                        yaw = angle - (M_PI / 2.0) * std::floor((angle + M_PI / 4.0) / (M_PI / 2.0));
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

                        cv::Mat debug = cv_image->image.clone();

                        for (const CubeColor& cube : cubes_){

                                Detection detection;
                                detection.name = cube.name;
                                detection.publisher = cube.publisher;

                                cv::Mat mask = color_mask(hsv, cube);
                                detect(mask, kernel, detection);
                                if (!detection.found) continue;

                                double center_z;
                                if (!compute_center_z(detection, cv_depth->image, center_z)){
                                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s cube: no valid depth", detection.name.c_str());
                                        continue;
                                }
                                // Le centre de la silhouette est sur le rayon du centre du cube : on le coupe
                                // a la hauteur mesuree
                                detection.position = pixel_to_world(detection.u, detection.v, msg->width, msg->height, center_z);

                                if (!compute_yaw(detection, cv_depth->image, detection.yaw)){
                                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s cube: not enough points for orientation", detection.name.c_str());
                                        continue;
                                }

                                geometry_msgs::msg::PoseStamped pose;
                                pose.header.stamp = msg->header.stamp;
                                pose.header.frame_id = "map";
                                pose.pose.position.x = detection.position.x();
                                pose.pose.position.y = detection.position.y();
                                pose.pose.position.z = detection.position.z();
                                // Rotation de yaw autour de z
                                pose.pose.orientation.z = std::sin(detection.yaw / 2.0);
                                pose.pose.orientation.w = std::cos(detection.yaw / 2.0);
                                detection.publisher->publish(pose);

                                std::vector<std::vector<cv::Point>> contours = {detection.contour};
                                cv::drawContours(debug, contours, 0, cv::Scalar(0, 255, 255), 5);
                                cv::circle(debug, cv::Point(detection.u, detection.v), 15, cv::Scalar(255, 255, 255), -1);
                                char label[64];
                                std::snprintf(label, sizeof(label), "%s z=%dmm yaw=%ddeg", detection.name.c_str(),
                                              static_cast<int>(std::round(detection.position.z() * 1000)),
                                              static_cast<int>(std::round(detection.yaw * 180.0 / M_PI)));
                                cv::putText(debug, label, cv::Point(detection.u + 30, detection.v),
                                            cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255), 6);
                        }

                        debug_image_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", debug).toImageMsg());
                }

                std::vector<CubeColor> cubes_;
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
                int yaw_erode_px_;
                int yaw_min_points_;

                rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
                rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
                sensor_msgs::msg::Image::ConstSharedPtr last_depth_;
                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
};

int main(int argc, char** argv){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<CubeDetector>());
        rclcpp::shutdown();
        return 0;
}
