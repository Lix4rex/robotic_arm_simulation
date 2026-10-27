#include <algorithm>
#include <list>
#include <set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msgs/action/move_to_position.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::chrono_literals;

class MainController : public rclcpp::Node{

        using MoveToPosition = robot_msgs::action::MoveToPosition;
        using GoalHandleMoveToPosition = rclcpp_action::ClientGoalHandle<MoveToPosition>;

        public:
                MainController() : Node("main_controller"){

                        RCLCPP_INFO(this->get_logger(), "MainController is launching...");

                        cube_order = this->declare_parameter<std::vector<std::string>>("cube_order", cube_order);
                        check_cube_order();

                        move_to_position_client_ = rclcpp_action::create_client<MoveToPosition>(this, "/arm_controller/move_to_position");

                        close_fingers_service_client_ = this->create_client<std_srvs::srv::Trigger>("/gripper/close");
                        open_fingers_service_client_ = this->create_client<std_srvs::srv::Trigger>("/gripper/open");

                        for (const auto& cube : cubes){
                                cubes_poses[cube] = Pose3D();
                                cube_sub_[cube] = this->create_subscription<geometry_msgs::msg::PointStamped>(
                                        "/cube_detector/" + cube + "_cube", 10, 
                                        [this, cube](const geometry_msgs::msg::PointStamped::SharedPtr msg){
                                                color_sub_callback(cube, msg);
                                        }
                                );

                        }

                        tick();

                        RCLCPP_INFO(this->get_logger(), "MainController successfuly launched");
                }


        private:


                enum class State { IDLE, MOVE_TO, OPEN_FINGERS, CLOSE_FINGERS, CUBE_PLACED, WAIT_FOR_MOVE, FINISH };

                struct Action{
                        State state = State::IDLE;
                        float x_target = 0.0;
                        float y_target = 0.0;
                        float z_target = 0.0;
                };

                int nb_cube_placed = 0;
                float z_floor = 1.048;
                float z_cube = 0.06;

                std::list<Action> actions = {
                        {State::IDLE, 0.0, 0.0, 0.0},
                        {State::OPEN_FINGERS, 0.0, 0.0, 0.0}                        
                };

                // Tick State Machine

                void tick(){
                        
                        const Action action = actions.front();
                        actions.pop_front();

                        switch(action.state){
                                case State::FINISH:
                                        RCLCPP_INFO(this->get_logger(), "Toutes les étapes ont été réalisées");
                                        return;

                                case State::IDLE:
                                        waiting_server_timer = this->create_wall_timer(500ms, std::bind(&MainController::wait_for_servers, this));
                                        break;

                                case State::CUBE_PLACED:
                                        nb_cube_placed++;
                                        moved_cube = 0;
                                        if (nb_cube_placed >= 3){
                                                actions.push_back({State::MOVE_TO, 0.6, 0, 1.4});
                                                actions.push_back({State::FINISH, 0.0, 0.0, 0.0});
                                        } else {
                                                pick_and_place_cube();
                                        }
                                        tick();
                                        break;

                                case State::WAIT_FOR_MOVE:
                                        pick_and_place_cube();
                                        tick();
                                        break;

                                case State::OPEN_FINGERS:
                                        call_open_fingers_service();
                                        break;

                                case State::CLOSE_FINGERS:
                                        call_close_fingers_service();
                                        break;

                                case State::MOVE_TO:
                                        send_move_to_position_goal(action.x_target, action.y_target, action.z_target);
                                        break;

                                default:
                                        RCLCPP_ERROR(this->get_logger(), "Cas non défini");
                                        return;
                        }
                }


                // Color detection and update pick_and_place

                struct Pose3D {
                        float x = 0.0;
                        float y = 0.0;
                        float z = 0.0;
                        bool ready = false;
                };

                std::vector<std::string> cubes = {"red", "green", "blue"};
                std::map<std::string, Pose3D> cubes_poses;

                int moved_cube = 0;

                std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr> cube_sub_;
                void color_sub_callback(std::string color, const geometry_msgs::msg::PointStamped::SharedPtr msg){
                        cubes_poses[color].x = msg->point.x;
                        cubes_poses[color].y = msg->point.y;
                        cubes_poses[color].z = msg->point.z;
                        if (!cubes_poses[color].ready) cubes_poses[color].ready = true;
                }

                std::vector<std::string> cube_order = {"green", "blue", "red"};

                void check_cube_order(){
                        std::set<std::string> seen;
                        for (const auto& color : cube_order){
                                if (std::find(cubes.begin(), cubes.end(), color) == cubes.end() || !seen.insert(color).second){
                                        throw std::invalid_argument("cube_order invalide : couleur inconnue ou en double '" + color + "'");
                                }
                        }
                        if (cube_order.size() != cubes.size()){
                                throw std::invalid_argument("cube_order doit contenir exactement " + std::to_string(cubes.size()) + " couleurs");
                        }
                        std::string liste;
                        for (const auto & c : cube_order) liste += (liste.empty() ? "" : ", ") + c;
                        RCLCPP_INFO(this->get_logger(), "Ordre d'empilement : %s", liste.c_str());
                }

                bool pose_in_range(Pose3D *a, Pose3D *b){
                        float epsilon = 0.05;
                        return b->x - epsilon <= a->x && a->x <= b->x + epsilon && b->y - epsilon <= a->y && a->y <= b->y + epsilon;
                }

                void pick_and_place_cube(){

                        std::string color_to_pick = cube_order[nb_cube_placed];
                        Pose3D cube_pose = cubes_poses[color_to_pick];

                        float cube_offset = 0.16f;
                        std::list<Action> moved_cube_reverse_action;

                        float max_z = cube_pose.z;
                        std::string color_to_move = color_to_pick;

                        for (std::string color : cube_order){
                                if (color == color_to_pick) continue;

                                Pose3D cube_pose_other = cubes_poses[color];

                                if (cube_pose_other.z < cube_pose.z) continue;
                                if (cube_pose_other.z < max_z) continue;

                                if (pose_in_range(&cube_pose, &cube_pose_other)){
                                        max_z = cube_pose_other.z;
                                        color_to_move = color;
                                        moved_cube++;
                                }
                        }

                        if (color_to_move != color_to_pick){
                                RCLCPP_INFO(this->get_logger(), "Il faut dépiler le cube %s\n", color_to_move.c_str());

                                Pose3D cube_pose_other = cubes_poses[color_to_move];
                                actions.push_back({State::MOVE_TO, cube_pose_other.x, cube_pose_other.y, cube_pose_other.z + 0.155f});
                                actions.push_back({State::MOVE_TO, cube_pose_other.x, cube_pose_other.y, cube_pose_other.z});
                                actions.push_back({State::CLOSE_FINGERS, 0.0, 0.0, 0.0});
                                actions.push_back({State::MOVE_TO, cube_pose_other.x, cube_pose_other.y, cube_pose_other.z + 0.255f});
                                
                                actions.push_back({State::MOVE_TO, cube_pose_other.x, cube_pose_other.y + cube_offset, 1.3f + (moved_cube-1)*z_cube});
                                actions.push_back({State::MOVE_TO, cube_pose_other.x, cube_pose_other.y + cube_offset, z_floor + (moved_cube-1)*z_cube});
                                actions.push_back({State::OPEN_FINGERS, 0.0, 0.0, 0.0});
                                actions.push_back({State::MOVE_TO, cube_pose_other.x, cube_pose_other.y + cube_offset, 1.3f + (moved_cube-1)*z_cube});

                                actions.push_back({State::WAIT_FOR_MOVE, 0.0, 0.0, 0.0});

                                return;
                        }

                        actions.push_back({State::MOVE_TO, cube_pose.x, cube_pose.y, cube_pose.z + 0.155f});
                        actions.push_back({State::MOVE_TO, cube_pose.x, cube_pose.y, cube_pose.z});
                        actions.push_back({State::CLOSE_FINGERS, 0.0, 0.0, 0.0});
                        actions.push_back({State::MOVE_TO, cube_pose.x, cube_pose.y, cube_pose.z + 0.255f});
                        
                        actions.push_back({State::MOVE_TO, 0.6, 0, 1.3});
                        actions.push_back({State::MOVE_TO, 0.6, 0, z_floor + nb_cube_placed*z_cube});
                        actions.push_back({State::OPEN_FINGERS, 0.0, 0.0, 0.0});
                        actions.push_back({State::MOVE_TO, 0.6, 0, 1.3});

                        actions.push_back({State::CUBE_PLACED, 0.0, 0.0, 0.0});
                }


                // Actions

                rclcpp_action::Client<MoveToPosition>::SharedPtr move_to_position_client_;

                void send_move_to_position_goal(const float x_target, const float y_target, const float z_target){
                        if (!move_to_position_client_->wait_for_action_server()) {
                                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                                rclcpp::shutdown();
                        }

                        auto goal_msg = MoveToPosition::Goal();
                        goal_msg.x_target = x_target;
                        goal_msg.y_target = y_target;
                        goal_msg.z_target = z_target;

                        RCLCPP_INFO(this->get_logger(), "Sending move_to_position goal : %f %f %f", x_target, y_target, z_target);

                        auto send_goal_options = rclcpp_action::Client<MoveToPosition>::SendGoalOptions();
                        send_goal_options.goal_response_callback = std::bind(&MainController::move_to_position_response_callback, this, std::placeholders::_1);
                        send_goal_options.feedback_callback = std::bind(&MainController::move_to_position_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
                        send_goal_options.result_callback = std::bind(&MainController::move_to_position_result_callback, this, std::placeholders::_1);

                        move_to_position_client_->async_send_goal(goal_msg, send_goal_options);
                }

                void move_to_position_response_callback(const GoalHandleMoveToPosition::SharedPtr &goal_handle){
                        if (!goal_handle) {
                                RCLCPP_ERROR(this->get_logger(), "move_to_position goal rejected by server");
                                return;
                        }
                        RCLCPP_INFO(this->get_logger(), "move_to_position goal accepted by server, waiting for result");
                }

                void move_to_position_feedback_callback(GoalHandleMoveToPosition::SharedPtr, const std::shared_ptr<const MoveToPosition::Feedback> feedback){
                        
                }

                void move_to_position_result_callback(const GoalHandleMoveToPosition::WrappedResult & result){
                        if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result->success){
                                RCLCPP_ERROR(this->get_logger(), "MoveTo échoué (code %d), arrêt", static_cast<int>(result.code));
                                return;
                        }
                        RCLCPP_INFO(this->get_logger(), "MoveTo sucessfully done");
                        tick();
                }


                // Services

                rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr close_fingers_service_client_;
                rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr open_fingers_service_client_;

                void call_close_fingers_service(){
                        if (!close_fingers_service_client_->service_is_ready()){
                                RCLCPP_ERROR(this->get_logger(), "CloseFingersService isn't ready");
                                return;
                        }
                        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

                        RCLCPP_INFO(this->get_logger(), "Sending close_fingers request");

                        close_fingers_service_client_->async_send_request(request, std::bind(&MainController::close_fingers_response_callback, this, std::placeholders::_1));
                }

                void close_fingers_response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
                        auto response = future.get();
                        if (!response->success){
                                RCLCPP_ERROR(this->get_logger(), "Failed to close fingers : %s", response->message.c_str());
                                return;
                        }
                        RCLCPP_INFO(this->get_logger(), "Fingers closed : %s", response->message.c_str());
                        tick();
                }

                void call_open_fingers_service(){
                        if (!open_fingers_service_client_->service_is_ready()){
                                RCLCPP_ERROR(this->get_logger(), "OpenFingersService isn't ready");
                                return;
                        }
                        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

                        RCLCPP_INFO(this->get_logger(), "Sending open_fingers request");

                        open_fingers_service_client_->async_send_request(request, std::bind(&MainController::open_fingers_response_callback, this, std::placeholders::_1));
                }

                void open_fingers_response_callback(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
                        auto response = future.get();
                        if (!response->success){
                                RCLCPP_ERROR(this->get_logger(), "Failed to open fingers : %s", response->message.c_str());
                                return;
                        }
                        RCLCPP_INFO(this->get_logger(), "Fingers opened : %s", response->message.c_str());
                        tick();
                }


                rclcpp::TimerBase::SharedPtr waiting_server_timer;
                void wait_for_servers(){
                        std::vector<std::string> manquants;
                        if (!move_to_position_client_->action_server_is_ready()) manquants.push_back("move_to_position_action");
                        if (!close_fingers_service_client_->service_is_ready()) manquants.push_back(close_fingers_service_client_->get_service_name());
                        if (!open_fingers_service_client_->service_is_ready()) manquants.push_back(open_fingers_service_client_->get_service_name());

                        for (auto cube : cubes){
                                if (!cubes_poses[cube].ready) manquants.push_back(cube);
                        }

                        if (!manquants.empty()){
                                std::string liste;
                                for (const auto & m : manquants) liste += (liste.empty() ? "" : ", ") + m;
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "En attente de : %s", liste.c_str());
                                return;
                        }

                        waiting_server_timer->cancel();
                        RCLCPP_INFO(this->get_logger(), "Tous les serveurs sont prêts : c'est parti");
                        pick_and_place_cube();
                        tick();
                }

};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<MainController>());
        rclcpp::shutdown();
        return 0;
}