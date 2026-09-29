#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "robot_msgs/msg/target_position.hpp"

#include <Eigen/Dense>
#include <cmath>

using namespace std::chrono_literals;

class ArmController : public rclcpp::Node{

        public:
                ArmController() : Node("arm_controller"){

                        RCLCPP_INFO(this->get_logger(), "ArmController is launching...");


                        RCLCPP_INFO(this->get_logger(), "Creating Publishers...");

                        panda_finger_joint1_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                                "/model/panda/joint/panda_finger_joint1/cmd_pos", 10
                        );                        
                        panda_finger_joint2_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                                "/model/panda/joint/panda_finger_joint1/cmd_pos", 10
                        );                        
                        panda_joint1_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                                "/model/panda/joint/panda_joint1/cmd_pos", 10
                        );                        
                        panda_joint2_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                                "/model/panda/joint/panda_joint2/cmd_pos", 10
                        );                        
                        panda_joint3_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                                "/model/panda/joint/panda_joint3/cmd_pos", 10
                        );                        
                        panda_joint4_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                                "/model/panda/joint/panda_joint4/cmd_pos", 10
                        );                        
                        panda_joint5_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                                "/model/panda/joint/panda_joint5/cmd_pos", 10
                        );                        
                        panda_joint6_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                                "/model/panda/joint/panda_joint6/cmd_pos", 10
                        );                        
                        panda_joint7_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                                "/model/panda/joint/panda_joint7/cmd_pos", 10
                        );

                        RCLCPP_INFO(this->get_logger(), "Publishers successfuly created");

                        target_position_sub_ = this->create_subscription<robot_msgs::msg::TargetPosition>(
                                "/target_position", 10,
                                std::bind(&ArmController::target_position_callback, this, std::placeholders::_1)
                        );

                        RCLCPP_INFO(this->get_logger(), "ArmController successfuly launched");
                }


        private:

                // Joint topics
                rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr panda_finger_joint1_pub_;
                rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr panda_finger_joint2_pub_;
                rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr panda_joint1_pub_;
                rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr panda_joint2_pub_;
                rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr panda_joint3_pub_;
                rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr panda_joint4_pub_;
                rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr panda_joint5_pub_;
                rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr panda_joint6_pub_;
                rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr panda_joint7_pub_;

                void publish_joint_positions(const Eigen::Matrix<double, 7, 1>& q){

                        std_msgs::msg::Float64 msg;

                        msg.data = q(0);
                        panda_joint1_pub_->publish(msg);

                        msg.data = q(1);
                        panda_joint2_pub_->publish(msg);

                        msg.data = q(2);
                        panda_joint3_pub_->publish(msg);

                        msg.data = q(3);
                        panda_joint4_pub_->publish(msg);

                        msg.data = q(4);
                        panda_joint5_pub_->publish(msg);

                        msg.data = q(5);
                        panda_joint6_pub_->publish(msg);

                        msg.data = q(6);
                        panda_joint7_pub_->publish(msg);
                }

                // Command topic
                rclcpp::Subscription<robot_msgs::msg::TargetPosition>::SharedPtr target_position_sub_;
                void target_position_callback(const robot_msgs::msg::TargetPosition::SharedPtr msg){
                        double x_target = msg->x_target;
                        double y_target = msg->y_target;
                        double z_target = msg->z_target;

                        Eigen::Vector3d target(
                                x_target,
                                y_target,
                                z_target
                        );                      
                        
                        // Configuration initiale// Configuration initiale
                        Eigen::Matrix<double, 7, 1> joint_angles;

                        joint_angles << -2.41,
                                        0.72,
                                        1.17,
                                        -3.07,
                                        -2.90,
                                        3.72,
                                        -1.08;

                        Eigen::Vector3d position = forward_kinematics(
                                joint_angles(0),
                                joint_angles(1),
                                joint_angles(2),
                                joint_angles(3),
                                joint_angles(4),
                                joint_angles(5),
                                joint_angles(6)
                        );

                        RCLCPP_INFO(
                                this->get_logger(),
                                "Position : x=%f y=%f z=%f",
                                position.x() + 0.2,
                                position.y(),
                                position.z() + 1.02
                        );

                        /*

                        Eigen::Matrix<double, 7, 1> q = inverse_kinematics(target, q0);

                        publish_joint_positions(q);

                        RCLCPP_INFO(
                                this->get_logger(),
                                "Solution IK:"
                        );

                        for (int i = 0; i < 7; i++)
                        {
                                RCLCPP_INFO(
                                this->get_logger(),
                                "q%d = %f",
                                i + 1,
                                q(i)
                                );
                        }*/
                }

                Eigen::Matrix4d T10(double alpha1){
                        Eigen::Matrix4d T;

                        T << std::cos(alpha1), -std::sin(alpha1), 0, 0,
                                std::sin(alpha1),  std::cos(alpha1), 0, 0,
                                0,                 0,                1, 0.333,
                                0,                 0,                0, 1;

                        return T;
                }


                Eigen::Matrix4d T21(double alpha2){
                        Eigen::Matrix4d T;

                        T << std::cos(alpha2), -std::sin(alpha2), 0, 0,
                                0,                 0,                1, 0,
                                -std::sin(alpha2), -std::cos(alpha2), 0, 0,
                                0,                 0,                0, 1;

                        return T;
                }


                Eigen::Matrix4d T32(double alpha3){
                        Eigen::Matrix4d T;

                        T << std::cos(alpha3), -std::sin(alpha3), 0, 0,
                                0,                 0,               -1, -0.316,
                                std::sin(alpha3),  std::cos(alpha3), 0, 0,
                                0,                 0,                0, 1;

                        return T;
                }


                Eigen::Matrix4d T43(double alpha4){
                        Eigen::Matrix4d T;

                        T << std::cos(alpha4), -std::sin(alpha4), 0, 0.0825,
                                0,                 0,               -1, 0,
                                std::sin(alpha4),  std::cos(alpha4), 0, 0,
                                0,                 0,                0, 1;

                        return T;
                }


                Eigen::Matrix4d T54(double alpha5){
                        Eigen::Matrix4d T;

                        T << std::cos(alpha5), -std::sin(alpha5), 0, -0.0825,
                                0,                 0,                1,  0.384,
                                -std::sin(alpha5), -std::cos(alpha5), 0,  0,
                                0,                 0,                0,  1;

                        return T;
                }

                Eigen::Matrix4d T65(double alpha6){
                        Eigen::Matrix4d T;

                        T << std::cos(alpha6), -std::sin(alpha6), 0, 0,
                                0,                 0,               -1, 0,
                                std::sin(alpha6),  std::cos(alpha6), 0, 0,
                                0,                 0,                0, 1;

                        return T;
                }


                Eigen::Matrix4d T76(double alpha7){
                        Eigen::Matrix4d T;

                        T << std::cos(alpha7), -std::sin(alpha7), 0, 0.088,
                                0,                 0,               -1, 0,
                                std::sin(alpha7),  std::cos(alpha7), 0, 0,
                                0,                 0,                0, 1;

                        return T;
                }

                Eigen::Matrix4d T87(){
                        Eigen::Matrix4d T;

                        T << 1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0.107,
                                0, 0, 0, 1;

                        return T;
                }

                Eigen::Vector3d forward_kinematics(double alpha1, double alpha2, double alpha3, double alpha4, double alpha5, double alpha6, double alpha7){
                        Eigen::Matrix4d T = T10(alpha1) * T21(alpha2) * T32(alpha3) * T43(alpha4) * T54(alpha5) * T65(alpha6) * T76(alpha7) * T87();

                        Eigen::Vector3d position;

                        position.x() = T(0, 3);
                        position.y() = T(1, 3);
                        position.z() = T(2, 3);

                        return position;
                }

                Eigen::Matrix<double, 3, 7> compute_jacobian(const Eigen::Matrix<double, 7, 1>& q){
                        Eigen::Matrix<double, 3, 7> J;

                        const double epsilon = 1e-6;

                        Eigen::Vector3d p0 = forward_kinematics(
                                q(0), q(1), q(2), q(3),
                                q(4), q(5), q(6)
                        );

                        for (int i = 0; i < 7; i++)
                        {
                                Eigen::Matrix<double, 7, 1> q_perturbed = q;
                                q_perturbed(i) += epsilon;

                                Eigen::Vector3d p1 = forward_kinematics(
                                        q_perturbed(0),
                                        q_perturbed(1),
                                        q_perturbed(2),
                                        q_perturbed(3),
                                        q_perturbed(4),
                                        q_perturbed(5),
                                        q_perturbed(6)
                                );

                                J.col(i) = (p1 - p0) / epsilon;
                        }

                        return J;
                }

                Eigen::Matrix<double, 7, 1> inverse_kinematics(const Eigen::Vector3d& target, Eigen::Matrix<double, 7, 1> q){
                        const int max_iterations = 1000;
                        const double tolerance = 1e-4;
                        const double alpha = 0.5;

                        for (int iteration = 0; iteration < max_iterations; iteration++)
                        {
                                // Position actuelle
                                Eigen::Vector3d current_position =
                                forward_kinematics(
                                        q(0), q(1), q(2), q(3),
                                        q(4), q(5), q(6)
                                );

                                // Erreur
                                Eigen::Vector3d error = target - current_position;

                                // Si on est suffisamment proche
                                if (error.norm() < tolerance)
                                {
                                        RCLCPP_INFO(
                                                this->get_logger(),
                                                "IK converged after %d iterations",
                                                iteration
                                        );

                                        return q;
                                }

                                // Jacobienne
                                Eigen::Matrix<double, 3, 7> J = compute_jacobian(q);

                                double lambda = 0.01;

                                Eigen::Matrix3d damping =
                                lambda * lambda * Eigen::Matrix3d::Identity();

                                Eigen::Matrix<double, 7, 3> J_pseudo_inverse =
                                J.transpose() *
                                (J * J.transpose() + damping).inverse();
                                
                                // Variation des angles
                                Eigen::Matrix<double, 7, 1> delta_q =
                                alpha * J_pseudo_inverse * error;

                                // Mise à jour
                                q += delta_q;
                        }

                        RCLCPP_WARN(
                                this->get_logger(),
                                "IK did not converge"
                        );

                        return q;
                }

};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<ArmController>());
        rclcpp::shutdown();
        return 0;
}