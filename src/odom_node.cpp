#include "stream.h"

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
std::shared_ptr<Stream> sp;
char send_buf[32];
char recv_buf[32];

ros::Subscriber vel_sub_;
ros::Publisher poly_pub_;
ros::Publisher odom_pub_;

void CmdVelCallback(const geometry_msgs::Twist &twist_aux) {
    if (twist_aux.linear.x > 0.2)
        twist_aux.linear.x = 0.2;
    if (twist_aux.angular.z > 2.0)
        twist_aux.angular.z = 2.0;
    // 速度
    int DesireL = twist_aux.linear.x * UNIT;
    int DesireR = twist_aux.linear.x * UNIT;
    // 角速度 rad/s
    // 本次要转动的角度 theta = DesireAngVelo * SAMPLE_TIME
    // Theta = dis / base   dis = theta * base
    // 每个轮子移动距离 d = dis/2 = theta * base / 2
    int DiffDis = (twist_aux.angular.z/3.14*180.0) * SAMPLE_TIME * WHEEL_BASE / 2.0 * UNIT;
    DesireL += DiffDis;
    DesireR -= DiffDis;
    memset(send_buf, 0, sizeof(send_buf));
    sprintf(send_buf, "$%+06d,%+06d\n", DesireL, DesireR);
    sp->Write(reinterpret_cast<uint8_t*>(send_buf), strlen(send_buf));
    ROS_INFO("send: %s", send_buf);
}

void SerialRecvTask() {
    tf::TransformBroadcaster odom_broadcaster_;

    ros::Time current_time = ros::Time::now();
    ros::Time last_time = ros::Time::now();
    while(ros::ok()) {
        memset(recv_buf, 0, sizeof(recv_buf));
        int ret = sp->ReadLine(reinterpret_cast<uint8_t*>(recv_buf), sizeof(recv_buf));

        if (recv_buf[0] == '#') {
            int l = 0, r = 0, voltage = 0;
            if (15 == ret) {
                sscanf(recv_buf, "#%d,%d\n", &l, &r);
                //ROS_INFO("Encoder Report: left=%d right=%d", l, r);
            } else if (8 == ret) {
                sscanf(recv_buf, "#%d\n", &voltage);
                ROS_INFO("Voltage Report: %.2fV", voltage/100.0);
            } else {
                ROS_INFO("Length not valid, recv: %s", recv_buf);
            }
        } else {
            ROS_INFO("Check failed, recv: %s", recv_buf);
        }
    }
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

int main(int argc, char** argv) {
    sp.reset(Stream::Serial("/dev/ttyTHS1", 460800));

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
