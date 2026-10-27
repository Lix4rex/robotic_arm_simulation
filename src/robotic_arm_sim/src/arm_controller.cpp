#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"
#include "robot_msgs/action/move_to_position.hpp"

#include <Eigen/Dense>
#include <cmath>

using namespace std::chrono_literals;

using MoveToPosition = robot_msgs::action::MoveToPosition;
using GoalHandleMoveToPosition = rclcpp_action::ServerGoalHandle<MoveToPosition>;

class ArmController : public rclcpp::Node{

        public:
                ArmController() : Node("arm_controller"){

                        RCLCPP_INFO(this->get_logger(), "ArmController is launching...");

                        max_joint_velocity_ = this->declare_parameter<double>("max_joint_velocity", 0.8);
                        min_trajectory_duration_ = this->declare_parameter<double>("min_trajectory_duration", 1.0);

                        // Position de depart des joints dans Gazebo (joint 4 en butee haute)
                        q_current_ << 0.0, 0.0, 0.0, -0.0698, 0.0, 0.0, 0.0;
                        q_start_ = q_current_;
                        q_goal_ = q_current_;

                        RCLCPP_INFO(this->get_logger(), "Creating Publishers...");

                        panda_finger_joint1_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                                "/model/panda/joint/panda_finger_joint1/cmd_pos", 10
                        );                        
                        panda_finger_joint2_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                                "/model/panda/joint/panda_finger_joint2/cmd_pos", 10
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

                        move_to_position_server_ = rclcpp_action::create_server<MoveToPosition>(
                                this, "/arm_controller/move_to_position",
                                std::bind(&ArmController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                                std::bind(&ArmController::handle_cancel, this, std::placeholders::_1),
                                std::bind(&ArmController::handle_accepted, this, std::placeholders::_1)
                        );

                        trajectory_timer_ = this->create_wall_timer(
                                10ms, std::bind(&ArmController::trajectory_callback, this)
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

                // Trajectory
                double max_joint_velocity_;
                double min_trajectory_duration_;
                Eigen::Matrix<double, 7, 1> q_current_;
                Eigen::Matrix<double, 7, 1> q_start_;
                Eigen::Matrix<double, 7, 1> q_goal_;
                std::chrono::steady_clock::time_point trajectory_start_;
                double trajectory_duration_ = 0.0;
                bool trajectory_active_ = false;
                rclcpp::TimerBase::SharedPtr trajectory_timer_;

                // Interpolation de q_start_ vers q_goal_ avec un profil quintique :
                // vitesse et acceleration nulles au depart et a l'arrivee, donc pas d'a-coup
                void start_trajectory(const Eigen::Matrix<double, 7, 1>& q_goal){
                        q_start_ = q_current_;
                        q_goal_ = q_goal;

                        // Vitesse max du profil quintique = 1.875 * deplacement / duree
                        const double max_delta = (q_goal_ - q_start_).cwiseAbs().maxCoeff();
                        trajectory_duration_ = std::max(1.875 * max_delta / max_joint_velocity_, min_trajectory_duration_);
                        trajectory_start_ = std::chrono::steady_clock::now();
                        trajectory_active_ = true;

                        RCLCPP_INFO(this->get_logger(), "Trajectory started (%.2f s)", trajectory_duration_);
                }

                void trajectory_callback(){
                        if (!trajectory_active_) return;

                        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - trajectory_start_).count();
                        const double tau = std::min(elapsed / trajectory_duration_, 1.0);
                        const double s = tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau);

                        q_current_ = q_start_ + s * (q_goal_ - q_start_);
                        publish_joint_positions(q_current_);

                        if (!active_goal_) {
                                if (tau >= 1.0) trajectory_active_ = false;
                                return;
                        }

                        // Annulation : on s'arrete a la position courante
                        if (active_goal_->is_canceling()){
                                trajectory_active_ = false;
                                auto result = std::make_shared<MoveToPosition::Result>();
                                result->success = false;
                                result->message = "Trajectory canceled";
                                active_goal_->canceled(result);
                                active_goal_.reset();
                                RCLCPP_INFO(this->get_logger(), "Trajectory canceled");
                                return;
                        }

                        auto feedback = std::make_shared<MoveToPosition::Feedback>();
                        feedback->progress = tau;
                        active_goal_->publish_feedback(feedback);

                        if (tau >= 1.0){
                                trajectory_active_ = false;
                                RCLCPP_INFO(this->get_logger(), "Trajectory finished");
                                auto result = std::make_shared<MoveToPosition::Result>();
                                result->success = true;
                                result->message = "Trajectory finished";
                                active_goal_->succeed(result);
                                active_goal_.reset();
                        }
                }

                // Action server
                rclcpp_action::Server<MoveToPosition>::SharedPtr move_to_position_server_;
                std::shared_ptr<GoalHandleMoveToPosition> active_goal_;

                rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToPosition::Goal> goal){
                        (void)uuid;
                        RCLCPP_INFO(this->get_logger(), "Received goal (%.3f, %.3f, %.3f)", goal->x_target, goal->y_target, goal->z_target);
                        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                }

                rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveToPosition> goal_handle){
                        (void)goal_handle;
                        RCLCPP_INFO(this->get_logger(), "Received cancel request");
                        return rclcpp_action::CancelResponse::ACCEPT;
                }

                // Le timer et ce callback tournent dans le meme executor (single-thread), pas besoin de mutex
                void handle_accepted(const std::shared_ptr<GoalHandleMoveToPosition> goal_handle){
                        // Un nouveau goal remplace celui en cours
                        if (active_goal_ && active_goal_->is_active()){
                                auto result = std::make_shared<MoveToPosition::Result>();
                                result->success = false;
                                result->message = "Preempted by a new goal";
                                active_goal_->abort(result);
                                RCLCPP_WARN(this->get_logger(), "Previous goal preempted");
                        }
                        active_goal_.reset();

                        const auto goal = goal_handle->get_goal();

                        // Cible dans le monde -> repere de la base (pose du panda dans panda_world.sdf)
                        double x_target = goal->x_target - 0.2;
                        double y_target = goal->y_target;
                        double z_target = goal->z_target - 1.025;

                        Eigen::Vector3d target(
                                x_target,
                                y_target,
                                z_target
                        );                      
                        
                        Eigen::Matrix<double, 7, 1> q0;

                        q0 << 0.0,
                              0.0,
                              0.0,
                              -1.5,
                              0.0,
                              1.0,
                              0.0;

                        bool converged = false;
                        Eigen::Matrix<double, 7, 1> q = inverse_kinematics(target, q0, converged);

                        if (!converged){
                                auto result = std::make_shared<MoveToPosition::Result>();
                                result->success = false;
                                result->message = "IK did not converge";
                                goal_handle->abort(result);
                                return;
                        }

                        active_goal_ = goal_handle;
                        start_trajectory(q);

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
                        }
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
                        const double c = std::cos(-M_PI / 4.0);
                        const double s = std::sin(-M_PI / 4.0);
                        Eigen::Matrix4d T;
                        T << c, -s, 0, 0,
                        s,  c, 0, 0,
                        0,  0, 1, 0.107,
                        0,  0, 0, 1;
                        return T;
                }

                // Point de saisie entre les doigts (TCP), 0.1034 m sous panda_hand
                Eigen::Matrix4d T_tcp(){
                        Eigen::Matrix4d T;
                        T << 1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 0.1034,
                        0, 0, 0, 1;
                        return T;
                }

                Eigen::Matrix4d forward_kinematics_matrix(const Eigen::Matrix<double, 7, 1>& q){
                        return T10(q(0)) * T21(q(1)) * T32(q(2)) * T43(q(3)) * T54(q(4)) * T65(q(5)) * T76(q(6))* T87() * T_tcp();
                }

                Eigen::Vector3d orientation_error(const Eigen::Matrix3d& R_current, const Eigen::Matrix3d& R_target){
                        Eigen::Vector3d error =
                                0.5 * (
                                R_current.col(0).cross(R_target.col(0)) +
                                R_current.col(1).cross(R_target.col(1)) +
                                R_current.col(2).cross(R_target.col(2))
                                );

                        return error;
                }

                Eigen::Matrix<double, 6, 7> compute_jacobian_6d(const Eigen::Matrix<double, 7, 1>& q){
                        Eigen::Matrix<double, 6, 7> J;

                        const double epsilon = 1e-6;

                        Eigen::Matrix4d T0 = forward_kinematics_matrix(q);

                        Eigen::Vector3d p0 = T0.block<3,1>(0,3);
                        Eigen::Matrix3d R0 = T0.block<3,3>(0,0);

                        for (int i = 0; i < 7; i++)
                        {
                                Eigen::Matrix<double, 7, 1> q_perturbed = q;
                                q_perturbed(i) += epsilon;

                                Eigen::Matrix4d T1 = forward_kinematics_matrix(q_perturbed);

                                Eigen::Vector3d p1 = T1.block<3,1>(0,3);
                                Eigen::Matrix3d R1 = T1.block<3,3>(0,0);

                                // Jacobien positionnel
                                J.block<3,1>(0,i) = (p1 - p0) / epsilon;

                                // Jacobien rotationnel
                                Eigen::Vector3d angular_error = orientation_error(R0, R1);

                                J.block<3,1>(3,i) = angular_error / epsilon;
                        }

                        return J;
                }

                Eigen::Matrix<double, 7, 1> inverse_kinematics(const Eigen::Vector3d& target_position, Eigen::Matrix<double, 7, 1>& q, bool& converged){
                        const int max_iterations = 1000;

                        const double position_tolerance = 1e-4;
                        const double orientation_tolerance = 1e-4;

                        const double alpha = 0.3;
                        const double lambda = 0.05;

                        Eigen::Matrix3d R_target;

                        R_target <<
                                1,  0,  0,
                                0, -1,  0,
                                0,  0, -1;

                        for (int iteration = 0; iteration < max_iterations; iteration++){
                                Eigen::Matrix4d T = forward_kinematics_matrix(q);

                                Eigen::Vector3d current_position = T.block<3,1>(0,3);
                                Eigen::Matrix3d current_rotation = T.block<3,3>(0,0);

                                Eigen::Vector3d position_error = target_position - current_position;
                                Eigen::Vector3d rotation_error = orientation_error(current_rotation, R_target);

                                Eigen::Matrix<double, 6, 1> error;

                                error.head<3>() = position_error;
                                error.tail<3>() = rotation_error;

                                if (position_error.norm() < position_tolerance && rotation_error.norm() < orientation_tolerance){
                                        RCLCPP_INFO(this->get_logger(), "IK converged after %d iterations",iteration);
                                        converged = true;
                                        return q;
                                }

                                Eigen::Matrix<double, 6, 7> J = compute_jacobian_6d(q);

                                Eigen::Matrix<double, 7, 6> J_pseudo_inverse = J.transpose() *(J * J.transpose() + lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity()).inverse();
                                Eigen::Matrix<double, 7, 1> delta_q = alpha * J_pseudo_inverse * error;

                                q += delta_q;

                                if (q(0) < -2.90) q(0) = -2.90;
                                if (q(0) >  2.90) q(0) =  2.90;

                                if (q(1) < -1.76) q(1) = -1.76;
                                if (q(1) >  1.76) q(1) =  1.76;

                                if (q(2) < -2.90) q(2) = -2.90;
                                if (q(2) >  2.90) q(2) =  2.90;

                                if (q(3) < -3.07) q(3) = -3.07;
                                if (q(3) > -0.70) q(3) = -0.70;

                                if (q(4) < -2.90) q(4) = -2.90;
                                if (q(4) >  2.90) q(4) =  2.90;

                                if (q(5) < -0.02) q(5) = -0.02;
                                if (q(5) >  3.75) q(5) =  3.75;

                                if (q(6) < -2.90) q(6) = -2.90;
                                if (q(6) >  2.90) q(6) =  2.90;
                        }

                        RCLCPP_WARN(
                                this->get_logger(),
                                "IK did not converge"
                        );

                        converged = false;
                        return q;
                }
};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<ArmController>());
        rclcpp::shutdown();
        return 0;
}