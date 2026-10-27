#include <list>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msgs/action/move_to_position.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class MainController : public rclcpp::Node{

        using MoveToPosition = robot_msgs::action::MoveToPosition;
        using GoalHandleMoveToPosition = rclcpp_action::ClientGoalHandle<MoveToPosition>;

        public:
                MainController() : Node("main_controller"){

                        RCLCPP_INFO(this->get_logger(), "MainController is launching...");

                        move_to_position_client_ = rclcpp_action::create_client<MoveToPosition>(this, "/arm_controller/move_to_position");

                        close_fingers_service_client_ = this->create_client<std_srvs::srv::Trigger>("/gripper/close");
                        open_fingers_service_client_ = this->create_client<std_srvs::srv::Trigger>("/gripper/open");


                        waiting_server_timer = this->create_wall_timer(500ms, std::bind(&MainController::wait_for_servers, this));

                        RCLCPP_INFO(this->get_logger(), "MainController successfuly launched");
                }


        private:


                enum class State { IDLE, MOVE_TO, OPEN_FINGERS, CLOSE_FINGERS, FINISH };

                struct Action{
                        State state = State::IDLE;
                        float x_target = 0.0;
                        float y_target = 0.0;
                        float z_target = 0.0;
                };

                std::list<Action> actions = {
                        {State::IDLE, 0.0, 0.0, 0.0},
                        {State::OPEN_FINGERS, 0.0, 0.0, 0.0},

                        {State::MOVE_TO, 0.8, -0.25, 1.2},
                        {State::MOVE_TO, 0.8, -0.25, 1.045},
                        {State::CLOSE_FINGERS, 0.0, 0.0, 0.0},
                        {State::MOVE_TO, 0.8, -0.25, 1.3},

                        {State::MOVE_TO, 0.6, 0, 1.3},
                        {State::MOVE_TO, 0.6, 0, 1.048},
                        {State::OPEN_FINGERS, 0.0, 0.0, 0.0},
                        {State::MOVE_TO, 0.6, 0, 1.3},

                        {State::MOVE_TO, 0.8, 0.25, 1.2},
                        {State::MOVE_TO, 0.8, 0.25, 1.045},
                        {State::CLOSE_FINGERS, 0.0, 0.0, 0.0},
                        {State::MOVE_TO, 0.8, 0.25, 1.3},

                        {State::MOVE_TO, 0.6, 0, 1.3},
                        {State::MOVE_TO, 0.6, 0, 1.108},
                        {State::OPEN_FINGERS, 0.0, 0.0, 0.0},
                        {State::MOVE_TO, 0.6, 0, 1.3},

                        {State::MOVE_TO, 0.8, 0, 1.2},
                        {State::MOVE_TO, 0.8, 0, 1.045},
                        {State::CLOSE_FINGERS, 0.0, 0.0, 0.0},
                        {State::MOVE_TO, 0.8, 0, 1.3},

                        {State::MOVE_TO, 0.6, 0, 1.3},
                        {State::MOVE_TO, 0.6, 0, 1.168},
                        {State::OPEN_FINGERS, 0.0, 0.0, 0.0},
                        {State::MOVE_TO, 0.6, 0, 1.3},

                        {State::MOVE_TO, 0.6, 0, 1.4},
                        
                        {State::FINISH, 0.0, 0.0, 0.0}
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

                        if (!manquants.empty()){
                                std::string liste;
                                for (const auto & m : manquants) liste += (liste.empty() ? "" : ", ") + m;
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "En attente de : %s", liste.c_str());
                                return;
                        }

                        waiting_server_timer->cancel();
                        RCLCPP_INFO(this->get_logger(), "Tous les serveurs sont prêts : c'est parti");
                        tick();
                }

};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<MainController>());
        rclcpp::shutdown();
        return 0;
}