#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"

#include <chrono>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

using SteadyClock = std::chrono::steady_clock;

class GripperController : public rclcpp::Node{

        public:
                GripperController() : Node("gripper_controller"){

                        RCLCPP_INFO(this->get_logger(), "GripperController is launching...");

                        cubes_ = this->declare_parameter<std::vector<std::string>>(
                                "cubes", {"blue_cube", "red_cube", "green_cube", "yellow_cube", "cyan_cube", "magenta_cube"}
                        );
                        open_position_ = this->declare_parameter<double>("open_position", 0.04);
                        close_position_ = this->declare_parameter<double>("close_position", 0.0);
                        close_timeout_ = this->declare_parameter<double>("close_timeout", 3.0);
                        contact_timeout_ = this->declare_parameter<double>("contact_timeout", 0.2);
                        release_clearance_ = this->declare_parameter<double>("release_clearance", 0.005);
                        release_timeout_ = this->declare_parameter<double>("release_timeout", 1.0);

                        RCLCPP_INFO(this->get_logger(), "Creating Publishers...");

                        panda_finger_joint1_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                                "/model/panda/joint/panda_finger_joint1/cmd_pos", 10
                        );
                        panda_finger_joint2_pub_ = this->create_publisher<std_msgs::msg::Float64>(
                                "/model/panda/joint/panda_finger_joint2/cmd_pos", 10
                        );

                        for (const auto& cube : cubes_){
                                attach_pubs_[cube] = this->create_publisher<std_msgs::msg::Empty>(
                                        "/panda/" + cube + "/attach", 10
                                );
                                detach_pubs_[cube] = this->create_publisher<std_msgs::msg::Empty>(
                                        "/panda/" + cube + "/detach", 10
                                );
                                // Etat reel de la liaison, publie par le plugin DetachableJoint
                                state_subs_.push_back(this->create_subscription<std_msgs::msg::String>(
                                        "/panda/" + cube + "/state", 10,
                                        [this, cube](const std_msgs::msg::String::SharedPtr msg){
                                                std::lock_guard<std::mutex> lock(mutex_);
                                                if (msg->data == "attached") attached_cubes_.insert(cube);
                                                else attached_cubes_.erase(cube);
                                        }
                                ));
                                contact_subs_.push_back(this->create_subscription<ros_gz_interfaces::msg::Contacts>(
                                        "/panda/" + cube + "/contacts", 10,
                                        [this, cube](const ros_gz_interfaces::msg::Contacts::SharedPtr msg){
                                                contacts_callback(cube, msg);
                                        }
                                ));
                        }

                        RCLCPP_INFO(this->get_logger(), "Publishers successfuly created");

                        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                                "/joint_states", 10,
                                std::bind(&GripperController::joint_state_callback, this, std::placeholders::_1)
                        );

                        // Les services attendent les contacts : ils tournent dans un groupe a part
                        // pour que les callbacks de contact continuent d'etre executes en parallele.
                        services_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

                        close_srv_ = this->create_service<std_srvs::srv::Trigger>(
                                "/gripper/close",
                                std::bind(&GripperController::close_callback, this, std::placeholders::_1, std::placeholders::_2),
                                rclcpp::ServicesQoS(), services_group_
                        );
                        open_srv_ = this->create_service<std_srvs::srv::Trigger>(
                                "/gripper/open",
                                std::bind(&GripperController::open_callback, this, std::placeholders::_1, std::placeholders::_2),
                                rclcpp::ServicesQoS(), services_group_
                        );
                        reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
                                "/gripper/reset",
                                std::bind(&GripperController::reset_callback, this, std::placeholders::_1, std::placeholders::_2),
                                rclcpp::ServicesQoS(), services_group_
                        );

                        RCLCPP_INFO(this->get_logger(), "GripperController successfuly launched");
                }


        private:

                struct FingerContacts{
                        SteadyClock::time_point left;
                        SteadyClock::time_point right;
                };

                std::vector<std::string> cubes_;
                double open_position_;
                double close_position_;
                double close_timeout_;
                double contact_timeout_;
                double release_clearance_;
                double release_timeout_;

                // Joint topics
                rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr panda_finger_joint1_pub_;
                rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr panda_finger_joint2_pub_;

                // DetachableJoint topics
                std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> attach_pubs_;
                std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr> detach_pubs_;

                std::vector<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr> contact_subs_;
                std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> state_subs_;
                rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

                rclcpp::CallbackGroup::SharedPtr services_group_;
                rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_srv_;
                rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_srv_;
                rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

                std::mutex mutex_;
                std::map<std::string, FingerContacts> last_contacts_;
                // Cubes attaches a la main : mis a jour par ce noeud et par l'etat du plugin
                std::set<std::string> attached_cubes_;
                double finger1_position_ = 0.0;
                double finger2_position_ = 0.0;
                bool finger_positions_received_ = false;

                void publish_fingers(double position){
                        std_msgs::msg::Float64 msg;
                        msg.data = position;
                        panda_finger_joint1_pub_->publish(msg);
                        panda_finger_joint2_pub_->publish(msg);
                }

                void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
                        std::lock_guard<std::mutex> lock(mutex_);
                        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++){
                                if (msg->name[i] == "panda_finger_joint1") finger1_position_ = msg->position[i];
                                if (msg->name[i] == "panda_finger_joint2") finger2_position_ = msg->position[i];
                        }
                        finger_positions_received_ = true;
                }

                void contacts_callback(const std::string& cube, const ros_gz_interfaces::msg::Contacts::SharedPtr msg){
                        const auto now = SteadyClock::now();
                        std::lock_guard<std::mutex> lock(mutex_);

                        for (const auto& contact : msg->contacts){
                                for (const auto* name : {&contact.collision1.name, &contact.collision2.name}){
                                        if (name->find("leftfinger") != std::string::npos) last_contacts_[cube].left = now;
                                        if (name->find("rightfinger") != std::string::npos) last_contacts_[cube].right = now;
                                }
                        }
                }

                // Renvoie le cube touche par les deux doigts, ou une chaine vide
                std::string cube_between_fingers(){
                        const auto now = SteadyClock::now();
                        const auto max_age = std::chrono::duration<double>(contact_timeout_);
                        std::lock_guard<std::mutex> lock(mutex_);

                        for (const auto& [cube, contacts] : last_contacts_){
                                if (now - contacts.left < max_age && now - contacts.right < max_age){
                                        return cube;
                                }
                        }
                        return "";
                }

                void close_callback(
                        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response){

                        {
                                std::lock_guard<std::mutex> lock(mutex_);
                                if (!attached_cubes_.empty()){
                                        response->success = false;
                                        response->message = *attached_cubes_.begin() + " is already grasped";
                                        return;
                                }
                        }

                        RCLCPP_INFO(this->get_logger(), "Closing gripper...");
                        publish_fingers(close_position_);

                        const auto deadline = SteadyClock::now() + std::chrono::duration<double>(close_timeout_);
                        std::string cube;
                        while (cube.empty() && SteadyClock::now() < deadline && rclcpp::ok()){
                                std::this_thread::sleep_for(20ms);
                                cube = cube_between_fingers();
                        }

                        if (cube.empty()){
                                RCLCPP_WARN(this->get_logger(), "No cube between both fingers, reopening gripper");
                                publish_fingers(open_position_);
                                response->success = false;
                                response->message = "No cube touched by both fingers";
                                return;
                        }

                        attach_pubs_[cube]->publish(std_msgs::msg::Empty());

                        // Le cube est solidaire de la main : on arrete de serrer en figeant les doigts
                        // la ou ils sont, sinon ils s'enfoncent dans le cube et le coincent au relachement
                        double finger1, finger2;
                        bool known;
                        {
                                std::lock_guard<std::mutex> lock(mutex_);
                                attached_cubes_.insert(cube);
                                finger1 = finger1_position_;
                                finger2 = finger2_position_;
                                known = finger_positions_received_;
                        }
                        if (known){
                                std_msgs::msg::Float64 msg;
                                msg.data = finger1;
                                panda_finger_joint1_pub_->publish(msg);
                                msg.data = finger2;
                                panda_finger_joint2_pub_->publish(msg);
                        }

                        RCLCPP_INFO(this->get_logger(), "%s grasped", cube.c_str());
                        response->success = true;
                        response->message = cube + " grasped";
                }

                void open_callback(
                        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response){

                        std::set<std::string> attached;
                        double finger1_start, finger2_start;
                        {
                                std::lock_guard<std::mutex> lock(mutex_);
                                attached = attached_cubes_;
                                finger1_start = finger1_position_;
                                finger2_start = finger2_position_;
                        }

                        RCLCPP_INFO(this->get_logger(), "Opening gripper...");
                        publish_fingers(open_position_);

                        // On ouvre d'abord, cube encore attache, et on ne detache qu'une fois les doigts
                        // ecartes : le cube tombe alors sans frotter contre un doigt
                        if (!attached.empty()){
                                const auto deadline = SteadyClock::now() + std::chrono::duration<double>(release_timeout_);
                                while (SteadyClock::now() < deadline && rclcpp::ok()){
                                        {
                                                std::lock_guard<std::mutex> lock(mutex_);
                                                if (finger1_position_ > finger1_start + release_clearance_ &&
                                                    finger2_position_ > finger2_start + release_clearance_) break;
                                        }
                                        std::this_thread::sleep_for(20ms);
                                }

                        }

                        // On detache tous les cubes, meme ceux qu'on ne croit pas attaches :
                        // open doit toujours lacher, quel que soit l'etat interne du noeud
                        for (const auto& cube : cubes_){
                                detach_pubs_[cube]->publish(std_msgs::msg::Empty());
                        }
                        {
                                std::lock_guard<std::mutex> lock(mutex_);
                                attached_cubes_.clear();
                        }

                        std::string released;
                        for (const auto& cube : attached){
                                released += (released.empty() ? "" : ", ") + cube;
                        }
                        if (!released.empty()) RCLCPP_INFO(this->get_logger(), "%s released", released.c_str());

                        response->success = true;
                        response->message = released.empty() ? "Gripper opened" : released + " released";
                }

                // Au reset de Gazebo, le plugin DetachableJoint est recree et re-attache tous les cubes
                // a la main (comme au demarrage) : on les detache et on repart d'un etat vide
                void reset_callback(
                        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response){

                        for (const auto& cube : cubes_){
                                detach_pubs_[cube]->publish(std_msgs::msg::Empty());
                        }
                        publish_fingers(open_position_);
                        {
                                std::lock_guard<std::mutex> lock(mutex_);
                                attached_cubes_.clear();
                                last_contacts_.clear();
                        }

                        RCLCPP_INFO(this->get_logger(), "GripperController reset");
                        response->success = true;
                        response->message = "Gripper reset";
                }
};

int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::executors::MultiThreadedExecutor executor;
        auto node = std::make_shared<GripperController>();
        executor.add_node(node);
        executor.spin();
        rclcpp::shutdown();
        return 0;
}
