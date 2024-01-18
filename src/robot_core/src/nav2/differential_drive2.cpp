#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class FramePublisher : public rclcpp::Node
{
    public:
        FramePublisher()
        : Node("icarus_tf2_broadcaster")
        {   
            //initialize broadcaster
            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            //auto Subscription=this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&FramePublisher::make_transforms,this,std::placeholders::_1));

            odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom_icarus", 10);
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("robot_state_publisher",10,std::bind(&FramePublisher::make_transforms, this, std::placeholders::_1));
        }


    private:
        void make_transforms( const std::shared_ptr<nav_msgs::msg::Odometry> )
        {
             //set vars for computing distance traveled using given velocities
            double x = 0.0;
            double y = 0.0;
            double th = 0.0;


            //TODO: CHANGE THE VELOCITIES HERE TO NOT BE CONSTANT
            double vx = 0.1;
            double vy = -0.1;
            double vth = 0.1;


            //computing odometry 
            /*double dt = (current_time - last_time).toSec();
            double delta_x = (vx * cos(th) - vy * sin(th)) *dt;
            double delta_y = (vx * sin(th) - vy * sin(th)) *dt;
            double delta_th = vth * dt;*/

            //x += delta_x;
            //y += delta_y;
            //th += delta_th;

            geometry_msgs::msg::TransformStamped t;

            //set our messages
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "odom_icarus";
            t.child_frame_id = "base_link";

             //get x and y translation since robot will only move in 2d frame
            t.transform.translation.x = x;
            t.transform.translation.y = y;
            t.transform.translation.z = 0.0;

            //robot can only rotate around a single axis due to mechanum structure
            tf2::Quaternion q;
            q.setRPY(0, 0, th);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_broadcaster->sendTransform(t);

            //publish the odom message over ros
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = this->get_clock()->now();
            odom.header.frame_id = "odometry_icarus";

            //setting positions
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0;

            //setting velocities
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.angular.z = vth;

            //publish the message
            odom_pub->publish(odom);



        }
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
};

int main(int argc, char * argv[])
{
    auto logger = rclcpp::get_logger("logger");

    // Pass parameters and initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
    }
