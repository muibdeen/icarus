#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class joysub : public rclcpp::Node
{
    public:
    joysub() 
    : Node("joysub")
    {
        subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&joysub::topic_callback, this, _1));
    }
    private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
      // Print all axes values
        for (size_t i = 0; i < msg->axes.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Axis[%zu]=%.2f", i, msg->axes[i]);
        }

        // Print all button values
        for (size_t i = 0; i < msg->buttons.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Button[%zu]=%d", i, msg->buttons[i]);
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<joysub>());
  rclcpp::shutdown();
  return 0;
}
