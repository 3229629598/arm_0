#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using namespace std::placeholders;

class Fake_Joint_Pub : public rclcpp::Node
{
    public:
    Fake_Joint_Pub():Node("fake_js_pub_node")
    {
        RCLCPP_INFO(get_logger(),"Fake_js_pub_node is running.");
        fake_js_pub=this->create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);
        fake_jtp_sub=this->create_subscription<sensor_msgs::msg::JointState>("/jtp_data", 10, std::bind(&Fake_Joint_Pub::fake_jtp_callback, this, _1));
        fake_request_pub=this->create_publisher<std_msgs::msg::UInt8>("/arm_request",10);
        timer_ = this->create_wall_timer(10ms, std::bind(&Fake_Joint_Pub::timer_callback, this));
        js_msg.name.insert(js_msg.name.begin(), {"joint1", "joint2", "joint3", "joint4","joint5","joint6","joint7","joint8","joint9","joint10","joint11"});
        js_msg.position.assign({0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
        arm_request_msg.data=0;
    }
    ~Fake_Joint_Pub()
    {}

    private:
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState js_msg;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr fake_js_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr fake_jtp_sub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr fake_request_pub;
    std_msgs::msg::UInt8 arm_request_msg;

    void fake_jtp_callback(const sensor_msgs::msg::JointState::SharedPtr fake_jtp_data)
    {
        uint8_t joint_num=fake_jtp_data->position.size();
        for(int i=0;i<joint_num;i++)
        {
            js_msg.position[i]=fake_jtp_data->position[i];
        }
        if(fake_jtp_data->header.frame_id=="final")
        {
            if(arm_request_msg.data==1)
            {
                arm_request_msg.data=0;
            }
            else
            {
                arm_request_msg.data=1;
            }
            if(arm_request_msg.data==1)
            {
                js_msg.position[7]=0.6;js_msg.position[8]=0.6;js_msg.position[9]=0.6;js_msg.position[10]=0.6;
            }
            else
            {
                js_msg.position[7]=0.0;js_msg.position[8]=0.0;js_msg.position[9]=0.0;js_msg.position[10]=0.0;
            }
            // fake_request_pub->publish(arm_request_msg);
        }
    }
    void timer_callback()
    {
        
        js_msg.header.stamp=this->now();
        // js_msg.position.assign({0.6,0.6,0.6,0.6});
        fake_js_pub->publish(js_msg);
    }
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Fake_Joint_Pub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}