#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class CmdVelPublisher : public rclcpp::Node
{
public:
    CmdVelPublisher() : Node("cmd_vel_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&CmdVelPublisher::publish_cmd_vel, this));
    }

private:
    void publish_cmd_vel()
    {
        // Create a TwistStamped message
        auto msg = geometry_msgs::msg::TwistStamped();


        msg.header.stamp = this->get_clock()->now();

        // Set the linear velocities
        msg.twist.linear.x = 1.0; 
        msg.twist.linear.y = 0.0;
        msg.twist.linear.z = 0.0;

        // Set angular velocities
        msg.twist.angular.x = 0.0;
        msg.twist.angular.y = 0.0;
        msg.twist.angular.z = 0.5; 

        RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x = %.2f, angular.z = %.2f", 
                    msg.twist.linear.x, msg.twist.angular.z);

        publisher_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelPublisher>());
    rclcpp::shutdown();

    return 0;
}
