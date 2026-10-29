#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"
#include "robot_msgs/action/move_to_position.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>

using namespace std::chrono_literals;

using MoveToPosition = robot_msgs::action::MoveToPosition;
using GoalHandleMoveToPosition = rclcpp_action::ServerGoalHandle<MoveToPosition>;

class ArmController : public rclcpp::Node{

        public:
                ArmController() : Node("arm_controller"){

                        RCLCPP_INFO(this->get_logger(), "ArmController is launching...");

                        max_joint_velocity_ = this->declare_parameter<double>("max_joint_velocity", 0.8);
                        min_trajectory_duration_ = this->declare_parameter<double>("min_trajectory_duration", 1.0);
                        max_linear_velocity_ = this->declare_parameter<double>("max_linear_velocity", 0.2);
                        reset_cubes_ = this->declare_parameter<std::vector<std::string>>(
                                "reset_cubes", {"blue_cube", "red_cube", "green_cube", "yellow_cube", "cyan_cube", "magenta_cube"}
                        );
                        reset_area_x_ = this->declare_parameter<std::vector<double>>("reset_area_x", {0.35, 1.14});
                        reset_area_y_ = this->declare_parameter<std::vector<double>>("reset_area_y", {-0.6, 0.6});
                        reset_robot_xy_ = this->declare_parameter<std::vector<double>>("reset_robot_xy", {0.2, 0.0});
                        reset_reach_ = this->declare_parameter<std::vector<double>>("reset_reach", {0.3, 0.8});
                        reset_exclusion_zones_ = this->declare_parameter<std::vector<double>>("reset_exclusion_zones", {0.5, 0.0, 0.2});
                        reset_base_z_ = this->declare_parameter<double>("reset_base_z", 1.07);
                        cube_stack_step_ = this->declare_parameter<double>("cube_stack_step", 0.07);
                        reset_min_spacing_ = this->declare_parameter<double>("reset_min_spacing", 0.12);
                        reset_stack_probability_ = this->declare_parameter<double>("reset_stack_probability", 0.4);
                        reset_max_yaw_ = this->declare_parameter<double>("reset_max_yaw", M_PI / 4);

                        // Position de depart des joints dans Gazebo (joint 4 en butee haute)
                        q_initial_ << 0.0, 0.0, 0.0, -0.0698, 0.0, 0.0, 0.0;
                        q_current_ = q_initial_;
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

                        reset_service_ = this->create_service<std_srvs::srv::Trigger>(
                                "/arm_controller/reset",
                                std::bind(&ArmController::handle_reset, this, std::placeholders::_1, std::placeholders::_2)
                        );
                        set_pose_client_ = this->create_client<ros_gz_interfaces::srv::SetEntityPose>("/world/panda_world/set_pose");
                        gripper_reset_client_ = this->create_client<std_srvs::srv::Trigger>("/gripper/reset");

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
                Eigen::Matrix<double, 7, 1> q_initial_;
                Eigen::Matrix<double, 7, 1> q_current_;
                Eigen::Matrix<double, 7, 1> q_start_;
                Eigen::Matrix<double, 7, 1> q_goal_;
                std::chrono::steady_clock::time_point trajectory_start_;
                double trajectory_duration_ = 0.0;
                bool trajectory_active_ = false;
                rclcpp::TimerBase::SharedPtr trajectory_timer_;

                // Trajectoire cartesienne : la pince (TCP) suit une ligne droite de p_start_ a p_goal_
                // (repere de la base) et le yaw passe de theta_start_ a theta_goal_, pince toujours vers le bas
                double max_linear_velocity_;
                bool cartesian_ = false;
                Eigen::Vector3d p_start_;
                Eigen::Vector3d p_goal_;
                double theta_start_ = 0.0;
                double theta_goal_ = 0.0;

                // Interpolation de q_start_ vers q_goal_ avec un profil quintique :
                // vitesse et acceleration nulles au depart et a l'arrivee, donc pas d'a-coup
                void start_trajectory(const Eigen::Matrix<double, 7, 1>& q_goal){
                        q_start_ = q_current_;
                        q_goal_ = q_goal;
                        cartesian_ = false;

                        // Vitesse max du profil quintique = 1.875 * deplacement / duree
                        const double max_delta = (q_goal_ - q_start_).cwiseAbs().maxCoeff();
                        trajectory_duration_ = std::max(1.875 * max_delta / max_joint_velocity_, min_trajectory_duration_);
                        trajectory_start_ = std::chrono::steady_clock::now();
                        trajectory_active_ = true;

                        RCLCPP_INFO(this->get_logger(), "Joint trajectory started (%.2f s)", trajectory_duration_);
                }

                // Meme profil quintique, mais applique a la position et au yaw de la pince ;
                // les joints sont recalcules par IK a chaque pas (voir trajectory_callback)
                void start_cartesian_trajectory(const Eigen::Vector3d& p_goal, double theta_goal, const Eigen::Matrix<double, 7, 1>& q_goal){
                        const Eigen::Matrix4d T = forward_kinematics_matrix(q_current_);
                        p_start_ = T.block<3,1>(0,3);
                        theta_start_ = std::atan2(T(1,0), T(0,0));
                        p_goal_ = p_goal;
                        // Plus court chemin en yaw (meme orientation a 2*pi pres)
                        theta_goal_ = theta_start_ + std::remainder(theta_goal - theta_start_, 2.0 * M_PI);
                        q_start_ = q_current_;
                        q_goal_ = q_goal;
                        cartesian_ = true;

                        // Duree limitee par la vitesse lineaire de la pince et par la vitesse des joints
                        const double distance = (p_goal_ - p_start_).norm();
                        const double max_delta = std::max((q_goal_ - q_start_).cwiseAbs().maxCoeff(), std::abs(theta_goal_ - theta_start_));
                        trajectory_duration_ = std::max({
                                1.875 * distance / max_linear_velocity_,
                                1.875 * max_delta / max_joint_velocity_,
                                min_trajectory_duration_
                        });
                        trajectory_start_ = std::chrono::steady_clock::now();
                        trajectory_active_ = true;

                        RCLCPP_INFO(this->get_logger(), "Cartesian trajectory started (%.2f s, %.3f m)", trajectory_duration_, distance);
                }

                void trajectory_callback(){
                        if (!trajectory_active_) return;

                        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - trajectory_start_).count();
                        const double tau = std::min(elapsed / trajectory_duration_, 1.0);
                        const double s = tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau);

                        if (cartesian_){
                                const Eigen::Vector3d p = p_start_ + s * (p_goal_ - p_start_);
                                const double theta = theta_start_ + s * (theta_goal_ - theta_start_);

                                // IK amorcee par la consigne precedente : petit pas, donc convergence rapide
                                // et pas de saut de configuration du bras en cours de route
                                Eigen::Matrix<double, 7, 1> q = q_initial_;
                                bool converged = false;
                                inverse_kinematics(p, theta, q, converged);
                                if (!converged){
                                        stop_motion("IK did not converge along the Cartesian path");
                                        return;
                                }
                                q_current_ = q;
                        } else {
                                q_current_ = q_start_ + s * (q_goal_ - q_start_);
                        }
                        publish_joint_positions(q_current_);

                        if (!active_goal_) {
                                if (tau >= 1.0){
                                        trajectory_active_ = false;
                                        // Retour a la pose initiale du reset termine : on replace les cubes
                                        if (reset_request_header_) reset_cubes();
                                }
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
                        RCLCPP_INFO(this->get_logger(), "Received goal (%.3f, %.3f, %.3f, %.3f)", goal->x_target, goal->y_target, goal->z_target, goal->theta_target);
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

                        double theta_target = goal->theta_target;

                        Eigen::Vector3d target(
                                x_target,
                                y_target,
                                z_target
                        );

                        // IK de la cible amorcee par la consigne courante pour garder la meme configuration du bras.
                        bool converged = false;
                        //Eigen::Matrix<double, 7, 1> q = q_current_;
                        Eigen::Matrix<double, 7, 1> q = q_initial_;
                        inverse_kinematics(target, theta_target, q, converged);

                        // Repli sur une configuration "bras plie, pince vers le bas" (ex : depuis la pose initiale tendue)
                        if (!converged){
                                q << 0.0, 0.0, 0.0, -1.5, 0.0, 1.0, 0.0;
                                inverse_kinematics(target, theta_target, q, converged);
                        }

                        if (!converged){
                                auto result = std::make_shared<MoveToPosition::Result>();
                                result->success = false;
                                result->message = "IK did not converge";
                                goal_handle->abort(result);
                                return;
                        }

                        active_goal_ = goal_handle;

                        // Ligne droite seulement si la pince pointe deja vers le bas : sinon le yaw seul ne
                        // decrit pas l'orientation de depart (ex : pose initiale), on passe alors en joints
                        if (forward_kinematics_matrix(q_current_)(2,2) < -0.99){
                                start_cartesian_trajectory(target, theta_target, q);
                        } else {
                                start_trajectory(q);
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

                Eigen::Matrix<double, 7, 1> inverse_kinematics(const Eigen::Vector3d& target_position, const float theta, Eigen::Matrix<double, 7, 1>& q, bool& converged){
                        const int max_iterations = 1000;

                        const double position_tolerance = 1e-4;
                        const double orientation_tolerance = 1e-4;

                        const double alpha = 0.3;
                        const double lambda = 0.05;

                        Eigen::Matrix3d R_target;

                        R_target <<
                                std::cos(theta),  std::sin(theta),  0,
                                std::sin(theta), -std::cos(theta),  0,
                                        0      ,        0        , -1;

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
                                        // DEBUG : appelee a chaque pas (100 Hz) pendant une trajectoire cartesienne
                                        RCLCPP_DEBUG(this->get_logger(), "IK converged after %d iterations",iteration);
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
                

                // Reset service
                using Trigger = std_srvs::srv::Trigger;
                using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;

                rclcpp::Service<Trigger>::SharedPtr reset_service_;
                rclcpp::Client<Trigger>::SharedPtr gripper_reset_client_;
                rclcpp::Client<SetEntityPose>::SharedPtr set_pose_client_;
                std::shared_ptr<rmw_request_id_t> reset_request_header_;
                std::vector<std::string> reset_cubes_;
                std::vector<double> reset_area_x_;  // [min, max] de la zone de depot
                std::vector<double> reset_area_y_;
                std::vector<double> reset_robot_xy_;        // (x, y) de la base du robot
                std::vector<double> reset_reach_;           // [min, max] distance horizontale a la base du robot
                std::vector<double> reset_exclusion_zones_; // x, y, rayon par zone interdite (zone de depot...)
                double reset_base_z_;               // z du centre d'un cube pose sur la table
                double cube_stack_step_;            // ecart en z entre deux cubes empiles (taille + marge)
                double reset_min_spacing_;          // distance min entre deux piles
                double reset_stack_probability_;    // proba de poser un cube sur une pile existante
                double reset_max_yaw_;              // yaw aleatoire tire dans [-max, max] (rad), 0 = cubes alignes
                std::mt19937 rng_{std::random_device{}()};
                size_t pending_cube_poses_ = 0;
                bool cube_pose_failed_ = false;

                // Reset de la scene sans reset Gazebo : un reset Gazebo recree les plugins sans
                // Reset (DetachableJoint re-attache les cubes, les capteurs de contact ne publient plus).
                // Chaine : pince (detach + ouverture) -> retour du bras a la pose initiale -> cubes replaces.
                // Executor single-thread : on ne peut pas attendre les services dans le callback,
                // la reponse est donc envoyee plus tard (send_response) a la fin de la chaine.
                void handle_reset(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<Trigger::Request> request){
                        (void)request;

                        if (reset_request_header_){
                                send_reset_response(request_header, false, "Reset already in progress");
                                return;
                        }
                        if (!gripper_reset_client_->service_is_ready() || !set_pose_client_->service_is_ready()){
                                send_reset_response(request_header, false, "Gripper reset or Gazebo set_pose service not available");
                                return;
                        }
                        reset_request_header_ = request_header;

                        stop_motion("Aborted by reset");

                        RCLCPP_INFO(this->get_logger(), "Resetting gripper...");
                        gripper_reset_client_->async_send_request(
                                std::make_shared<Trigger::Request>(),
                                std::bind(&ArmController::gripper_reset_callback, this, std::placeholders::_1)
                        );
                }

                void gripper_reset_callback(rclcpp::Client<Trigger>::SharedFuture future){
                        const auto response = future.get();
                        if (!response->success){
                                finish_reset(false, "Gripper reset failed : " + response->message);
                                return;
                        }

                        // La suite (reset_cubes) est lancee par trajectory_callback a la fin du mouvement
                        RCLCPP_INFO(this->get_logger(), "Moving arm back to initial position...");
                        start_trajectory(q_initial_);
                }

                // Position atteignable par le bras, hors de son socle et hors des zones interdites
                bool is_reset_position_valid(double x, double y){
                        double dist_robot = std::hypot(x - reset_robot_xy_[0], y - reset_robot_xy_[1]);
                        if (dist_robot < reset_reach_[0] || dist_robot > reset_reach_[1]) return false;

                        for (size_t k = 0; k + 2 < reset_exclusion_zones_.size(); k += 3){
                                if (std::hypot(x - reset_exclusion_zones_[k], y - reset_exclusion_zones_[k + 1]) < reset_exclusion_zones_[k + 2]) return false;
                        }
                        return true;
                }

                void reset_cubes(){
                        RCLCPP_INFO(this->get_logger(), "Moving cubes to random positions...");
                        pending_cube_poses_ = reset_cubes_.size();
                        cube_pose_failed_ = false;

                        // Tirage aleatoire : chaque cube est pose soit sur la table a une position (x, y)
                        // libre, soit au sommet d'une pile existante. L'ordre est melange pour varier les piles.
                        struct Pile { double x, y; int height; };
                        std::vector<Pile> piles;
                        std::uniform_real_distribution<double> dist_x(reset_area_x_[0], reset_area_x_[1]);
                        std::uniform_real_distribution<double> dist_y(reset_area_y_[0], reset_area_y_[1]);
                        std::bernoulli_distribution stack_dist(reset_stack_probability_);
                        std::uniform_real_distribution<double> dist_yaw(-reset_max_yaw_, reset_max_yaw_);

                        std::vector<size_t> order(reset_cubes_.size());
                        std::iota(order.begin(), order.end(), 0);
                        std::shuffle(order.begin(), order.end(), rng_);

                        for (size_t i : order){
                                int pile_idx = -1;
                                if (!piles.empty() && stack_dist(rng_)){
                                        pile_idx = std::uniform_int_distribution<int>(0, piles.size() - 1)(rng_);
                                }
                                else {
                                        // Cherche une position assez loin des piles existantes
                                        for (int attempt = 0; attempt < 100 && pile_idx < 0; attempt++){
                                                double x = dist_x(rng_), y = dist_y(rng_);
                                                bool free = is_reset_position_valid(x, y) && std::all_of(piles.begin(), piles.end(), [&](const Pile& p){
                                                        return std::hypot(p.x - x, p.y - y) >= reset_min_spacing_;
                                                });
                                                if (free){
                                                        piles.push_back({x, y, 0});
                                                        pile_idx = piles.size() - 1;
                                                }
                                        }
                                        // Pas de place libre : on empile sur une pile au hasard
                                        if (pile_idx < 0) pile_idx = std::uniform_int_distribution<int>(0, piles.size() - 1)(rng_);
                                }
                                Pile& pile = piles[pile_idx];

                                auto request = std::make_shared<SetEntityPose::Request>();
                                request->entity.name = reset_cubes_[i];
                                request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
                                request->pose.position.x = pile.x;
                                request->pose.position.y = pile.y;
                                request->pose.position.z = reset_base_z_ + cube_stack_step_ * pile.height++;
                                // Rotation autour de z uniquement : le cube reste a plat
                                double yaw = dist_yaw(rng_);
                                request->pose.orientation.z = std::sin(yaw / 2);
                                request->pose.orientation.w = std::cos(yaw / 2);

                                const std::string cube = reset_cubes_[i];
                                set_pose_client_->async_send_request(request,
                                        [this, cube](rclcpp::Client<SetEntityPose>::SharedFuture future){
                                                if (!future.get()->success){
                                                        RCLCPP_ERROR(this->get_logger(), "set_pose failed for %s", cube.c_str());
                                                        cube_pose_failed_ = true;
                                                }
                                                if (--pending_cube_poses_ > 0) return;

                                                if (cube_pose_failed_) finish_reset(false, "Failed to move some cubes back");
                                                else finish_reset(true, "Gripper, arm and cubes reset");
                                        }
                                );
                        }
                }

                void finish_reset(bool success, const std::string& message){
                        if (success) RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
                        else RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
                        send_reset_response(reset_request_header_, success, message);
                        reset_request_header_.reset();
                }

                void send_reset_response(const std::shared_ptr<rmw_request_id_t>& request_header, bool success, const std::string& message){
                        Trigger::Response response;
                        response.success = success;
                        response.message = message;
                        reset_service_->send_response(*request_header, response);
                }

                // Arrete la trajectoire en cours et abandonne le goal actif
                void stop_motion(const std::string& reason){
                        if (active_goal_ && active_goal_->is_active()){
                                auto result = std::make_shared<MoveToPosition::Result>();
                                result->success = false;
                                result->message = reason;
                                active_goal_->abort(result);
                                RCLCPP_WARN(this->get_logger(), "Active goal aborted : %s", reason.c_str());
                        }
                        active_goal_.reset();
                        trajectory_active_ = false;
                }
};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<ArmController>());
        rclcpp::shutdown();
        return 0;
}