#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class ArmController : public rclcpp::Node{

        public:
                ArmController() : Node("arm_controller"){

                        RCLCPP_INFO(this->get_logger(), "ArmController is launching...");
                        RCLCPP_INFO(this->get_logger(), "ArmController has been successfuly launched...");
                }


        private:

};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<ArmController>());
        rclcpp::shutdown();
        return 0;
}