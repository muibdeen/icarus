#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include <nav_msgs/Odometry.h>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <sensor_msgs>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


    //create a nodehandle for new nodes
    //create a publisher from odom transform to baselink
    //define the tf.transformbroadcaster
class tf_transform : public rclcpp::Node
    {
        public:
            framePublisher()
            :Node("Robot_Publisher")
            {
                //create and get name of robot
                robot_name = this->declare_parameter<std::string>("robot_name","icarus");
                //initialize tf broadcast
                tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this)

                //subscribe to some type of pose topic and call the pose
                //create a callback on each message
                std::ostring stream;
                stream << "/" <<robot_name.c_str() << "/pose";
                std::string topic_name = stream.str(); 

                Subscription = this ->create_subscription<nav_msgs::Odometry>(
                    topic_name, 10,
                    std::bind(&framePublisher::robot_pose_handle, this, std::placeholders::_1));
            }

           /* odom_pub()
            :Node("Odom_publisher")
            {
            odom_pub=this->create_publisher<nav_msgs::Odometry>("odom", 20);
            }
        }*/

        private:
        void robot_pose_handle()
        {



            //set vars for computing distance traveled using given velocities
            double x = 0.0;
            double y = 0.0;
            double th = 0.0;


            //TODO: CHANGE THE VELOCITIES HERE TO NOT BE CONSTANT
            double vx = 0.1;
            double vy = -0.1;
            double vth = 0.1;

            rclcpp::Time current_time, last_time;

            //computing odometry 
            double dt = (current_time - last_time).toSec();
            double delta_x = (vx * cos(th) - vy * sin(th)) *dt;
            double delta_y = (vx * sin(th) - vy * sin(th)) *dt;
            double delta_th = vth * dt;

            x += delta_x;
            y += delta_y;
            th += delta_th;
            
            //read geometry messages and assign them to tf variables
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id="world";
            t.child_frame_id="base_link";

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

            //send information over
            tf_broadcaster ->sendTransform(t);


            //publish geometry message over ros
           /* nav_msgs::Odometry odom;
            odom.header.stamp = this->get_clock()->now()
            odom.header.frame_id="odom"
        

            //setting positions
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = z;
            odom.pose.pose.orientation = q;

            //setting starting velocities
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = vth

            //publsih odom message
            odom_pub -> (odom) ;*/

        }
          std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    };
    //create variables for x, y, and theta position coords

    //create variiables for velocity in the x y, and theta

    //track time

    //set refresh rate

    //set an infinite loop for running program
        //wait for new inputs to come in and track time that it occured

        //create variables holding computed odom (velocity=(pos2-pos1)/time)


        //transform everything into a quaternion due to 6dof odom

        //publish transform over to tf using odom message headers

        //send the transform 

        //publish odom messager using ROS

        //set positions

        //set velocities

        //publish message

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<framePublisher<());
    rclcpp::shutdown();
    return 0;

}