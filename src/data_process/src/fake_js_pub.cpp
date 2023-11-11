#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class fake_js_pub : public rclcpp::Node
{
    public:
    fake_js_pub():Node("fake_js_pub_node")
    {
        RCLCPP_INFO(get_logger(),"Fake_js_pub_node is running.");
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&fake_js_pub::timer_callback, this));
        js_msg.name.insert(js_msg.name.begin(), {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    }

    private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    size_t count_;
    sensor_msgs::msg::JointState js_msg;
    void timer_callback()
    {
        
        js_msg.header.stamp=this->now();
        js_msg.position.assign({0,0,0,0,0,0});
        publisher_->publish(js_msg);
    }
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fake_js_pub>());
    rclcpp::shutdown();
    return 0;
}