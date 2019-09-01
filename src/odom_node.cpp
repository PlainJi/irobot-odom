#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>
#include <thread>
#include <iomanip>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// send the excepted number of pulses of the left&right wheel
// $+12345,+12345\n

// recv the actual number of pulses of the left&right wheel
// #+00001,+00001\n
// recv the current battery voltage
// #+01234\n

#define WHEEL_BASE  (0.15f)                     //轮距 m
#define PERIMITER   (0.206f)			        //轮子周长 m
#define UNIT        (512*27/PERIMITER)	        //每米对应的编码器脉冲数
#define SAMPLE_TIME (0.005f)			        //编码器采样周期 5ms

using namespace std;
using namespace boost::asio;
std::shared_ptr<serial_port> sp_;

ros::Subscriber vel_sub_;
ros::Publisher poly_pub_;
ros::Publisher odom_pub_;

void CmdVelCallback(const geometry_msgs::Twist &twist_aux) {
    // 速度
    int DesireL = twist_aux.linear.x * UNIT;
    int DesireR = twist_aux.linear.x * UNIT;
    // 角速度 rad/s
    // 本次要转动的角度 theta = DesireAngVelo * SAMPLE_TIME
    // Theta = dis / base   dis = theta * base
    // 每个轮子移动距离 d = dis/2 = theta * base / 2
    int DiffDis = twist_aux.angular.z * SAMPLE_TIME * WHEEL_BASE / 2.0 * UNIT;
    DesireL -= DiffDis;
    DesireR += DiffDis;

    boost::asio::streambuf temp;
    char *p = const_cast<char*>(boost::asio::buffer_cast<const char*>(temp.data()));
    sprintf(p, "$%+06d,%+06d\n", DesireL, DesireR);
    write(*sp_, temp);
    std::cout << "send: " << p;
}

void SerialRecvTask() {
    int cnt = 0;
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;
    double dt = 0.0;
    tf::TransformBroadcaster odom_broadcaster_;

    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();
    while(ros::ok()) {
        boost::asio::streambuf temp;
        read_until(*sp_, temp, '\n');
        char *p = const_cast<char*>(boost::asio::buffer_cast<const char*>(temp.data()));

        if (p[0] == '#') {
            int l = 0, r = 0, voltage = 0;
            if (15 == strlen(p)) {
                sscanf(p, "#%d,%d\n", &l, &r);
                ROS_INFO("Encoder Report: left=%d right=%d", l, r);
            } else if (22 == strlen(p)) {
                sscanf(p, "#%d,%d\n#%d", &l, &r, &voltage);
                ROS_INFO("Encoder Report: left=%d right=%d", l, r);
                ROS_INFO("Voltage Report: %.2fV", voltage/100.0);
            }
        } else {
            ROS_INFO("Serial port communication failed! %s", p);
        }
    }   // end-while
}

void PolyPubTask() {
    ros::Rate r(10);

    while(ros::ok()) {
        //publish polygon messageBotInit
        geometry_msgs::Point32 point[4];
        // coordinates described in base_link frame
        point[0].x = -0.39;  point[0].y = -0.31;
        point[1].x = 0.39;   point[1].y = -0.31;
        point[2].x = 0.39;   point[2].y = 0.31;
        point[3].x = -0.39;  point[3].y = 0.31;

        ros::Time current_time = ros::Time::now();
        geometry_msgs::PolygonStamped poly;
        poly.header.stamp = current_time;
        poly.header.frame_id = "base_link";
        poly.polygon.points.push_back(point[0]);
        poly.polygon.points.push_back(point[1]);
        poly.polygon.points.push_back(point[2]);
        poly.polygon.points.push_back(point[3]);
        poly_pub_.publish(poly);

        r.sleep();
    }
}

int main(int argc, char** argv)
{
    io_service iosev_;
    sp_.reset(new serial_port(iosev_, "/dev/ttyTHS1"));
    sp_->set_option(serial_port::baud_rate(460800));
    sp_->set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp_->set_option(serial_port::parity(serial_port::parity::none));
    sp_->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp_->set_option(serial_port::character_size(8));
    iosev_.run();

    ros::init(argc, argv, "base_controller");
    ros::NodeHandle n_;
    odom_pub_ = n_.advertise<nav_msgs::Odometry>("/odom", 10);
    poly_pub_ = n_.advertise<geometry_msgs::PolygonStamped>("/polygon",10);
    vel_sub_ = n_.subscribe("/cmd_vel", 10, CmdVelCallback);

    std::thread recv_task_ = std::thread(std::bind(SerialRecvTask));
    std::thread poly_task_ = std::thread(std::bind(PolyPubTask));

    ros::spin();

    if (recv_task_.joinable()) {
        recv_task_.join();
        ROS_INFO("recv_task_ joined!");
    }
    if (poly_task_.joinable()) {
        poly_task_.join();
        ROS_INFO("poly_task_ joined!");
    }
}
