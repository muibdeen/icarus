#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

class CmdVelPublisher : public rclcpp::Node
{
public:
    CmdVelPublisher()
        : Node("cmd_vel_publisher")
    {
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Create a timer to periodically check for TF updates
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CmdVelPublisher::updateCmdVel, this));
    }

private:
    void updateCmdVel()
    {
        try
        {
            // Use the TF buffer to lookup the transform from base_link to odom
            geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_.lookupTransform("base_link", "/demo/odom", tf2::TimePointZero);

            // Use the transform information to compute cmd_vel (for demonstration purposes, replace this with your logic)
            double linear_velocity = 0.1;
            double angular_velocity = 0.05;

            // Create and publish cmd_vel message
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = linear_velocity;
            cmd_vel_msg.angular.z = angular_velocity;

            cmd_vel_pub_->publish(cmd_vel_msg);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        }
    }

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    tf2::BufferCore tf_buffer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelPublisher>());
    rclcpp::shutdown();
    return 0;
}
