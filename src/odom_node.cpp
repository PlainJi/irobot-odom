#include <iostream>
#include <iomanip>
#include <math.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace boost::asio;


void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
    speed_buf.vx = twist_aux.linear.x;
    speed_buf.vy = twist_aux.linear.y;
    speed_buf.vth = twist_aux.angular.z;
}


double x = 0.0;
double y = 0.0;
double th = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
double dt = 0.0;

int main(int argc, char** argv)
{
    io_service iosev;
    serial_port sp(iosev, "/dev/ttyUSB0");
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));

    ros::init(argc, argv, "base_controller");
    ros::NodeHandle n;

    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 100, cmd_velCallback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Publisher poly_pub = n.advertise<geometry_msgs::PolygonStamped>("polygon",10);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while(ros::ok())
    {
        current_time = ros::Time::now();

        read(sp, buffer(speed_buf_rev));

        if(CRC_verify(speed_buf_rev))
        {
             vx  = speed_buf_rev.vx;
             vy  = speed_buf_rev.vy;
             vth = speed_buf_rev.vth;

             ROS_INFO("vx  is %2f", vx);
             ROS_INFO("vy  is %2f", vy);
             ROS_INFO("vth is %2f", vth);

             /**compute odometry in a typical way given the velocities of the robot**/
             double dt = (current_time - last_time).toSec();
             double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
             double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
             double delta_th = vth * dt;

             x += delta_x;
             y += delta_y;
             th += delta_th;

             /***********first, we'll publish the transform over tf*************/
             geometry_msgs::TransformStamped odom_trans;
             odom_trans.header.stamp = current_time;
             odom_trans.header.frame_id = "odom";
             odom_trans.child_frame_id = "base_link";

             odom_trans.transform.translation.x = x;
             odom_trans.transform.translation.y = y;
             odom_trans.transform.translation.z = 0.0;
             odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

             // send the transform
             odom_broadcaster.sendTransform(odom_trans);


             /*********next, we'll publish the odometry message over ROS*******/
             nav_msgs::Odometry odom;
             odom.header.stamp = current_time;
             odom.header.frame_id = "odom";
             odom.child_frame_id = "base_link";

             // since all odometry is 6DOF we'll need a quaternion created from yaw
             geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

             //set the position
             odom.pose.pose.position.x = x;
             odom.pose.pose.position.y = y;
             odom.pose.pose.position.z = 0.0;
             odom.pose.pose.orientation = odom_quat;

             // set the velocity
             odom.twist.twist.linear.x = vx;
             odom.twist.twist.linear.y = vy;
             odom.twist.twist.angular.z = vth;

             odom_pub.publish(odom);


             /*******************publish polygon message***********************/
             geometry_msgs::Point32 point[4];
             // coordinates described in base_link frame
             point[0].x = -0.39;  point[0].y = -0.31;
             point[1].x = 0.39;   point[1].y = -0.31;
             point[2].x = 0.39;   point[2].y = 0.31;
             point[3].x = -0.39;  point[3].y = 0.31;

             geometry_msgs::PolygonStamped poly;
             poly.header.stamp = current_time;
             poly.header.frame_id = "base_link";
             poly.polygon.points.push_back(point[0]);
             poly.polygon.points.push_back(point[1]);
             poly.polygon.points.push_back(point[2]);
             poly.polygon.points.push_back(point[3]);

             poly_pub.publish(poly);
        }
        else
		ROS_INFO("Serial port communication failed!");
  

        write(sp, buffer(speed_buf, buffer_length));

        last_time = current_time;

        ros::spinOnce();
    }   // end-while

    iosev.run();

}