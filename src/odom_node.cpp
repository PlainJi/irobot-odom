#include "stream.h"
#include <math.h>
#include <iostream>
#include <thread>

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

// send the excepted number of pulses of the left&right wheel
// $+12345,+12345\n

// recv the actual number of pulses of the left&right wheel
// #+00001,+00001\n
// recv the current battery voltage
// #+01234\n

#define WHEEL_BASE      (0.20)                      //轮距 m
#define PERIMITER       (0.238)			            //轮子周长 m
#define UNIT            (512*27/PERIMITER)	        //每米对应的编码器脉冲数
#define CONTROL_TIME    (0.005)			            //编码器采样周期 5ms
#define REPOET_TIME     (0.1)                       //小车里程计上报周期
#define PI              (3.14159265358979)

using namespace std;
std::shared_ptr<Stream> sp;
char send_buf[32];
char recv_buf[32];

ros::Subscriber vel_sub;
ros::Publisher poly_pub;
ros::Publisher odom_pub;

void CmdVelCallback(const geometry_msgs::Twist &twist_aux) {
    double linear_speed = twist_aux.linear.x;
    double angular_speed = twist_aux.angular.z;
    if (linear_speed > 0.2)
        linear_speed = 0.2;
    else if (linear_speed < -0.2)
        linear_speed = -0.2;
    if (angular_speed > 2.0)
        angular_speed = 2.0;
    else if (angular_speed < -2.0)
        angular_speed = -2.0;

    // 每个control time内期望的脉冲数=速度(m/s) * 每米的脉冲数(个) * CONTROL_TIME
    int DesireL = linear_speed * UNIT * CONTROL_TIME;
    int DesireR = linear_speed * UNIT * CONTROL_TIME;
    // 角速度 rad/s
    // 本次要转动的角度 theta = DesireAngVelo * CONTROL_TIME
    // Theta = dis / base   dis = theta * base
    // 每个轮子移动距离 d = dis/2 = theta * base / 2
    int DiffDis = angular_speed * CONTROL_TIME * WHEEL_BASE / 2.0 * UNIT;
    DesireL += DiffDis;
    DesireR -= DiffDis;
    memset(send_buf, 0, sizeof(send_buf));
    sprintf(send_buf, "$%+06d,%+06d\n", DesireL, DesireR);
    ROS_INFO("desire   L=%d   R=%d   diff: %d", DesireL, DesireR, DiffDis);
    sp->Write(reinterpret_cast<uint8_t*>(send_buf), strlen(send_buf));
    ROS_INFO("send: %s", send_buf);
}

void SerialRecvTask() {
    double DisLeft=0, DisRight=0, Distance=0, DistanceDiff=0;
    double delta_x, delta_y, delta_theta=0, r=0;
    double x=0, y=0, th=0;
    double vx=0, vy=0, vth=0;
    tf::TransformBroadcaster odom_broadcaster;

    while(ros::ok()) {
        memset(recv_buf, 0, sizeof(recv_buf));
        int ret = sp->ReadLine(reinterpret_cast<uint8_t*>(recv_buf), sizeof(recv_buf));

        if (recv_buf[0] == '#') {
            int l = 0, r = 0, voltage = 0;
            if (15 == ret) {
                sscanf(recv_buf, "#%d,%d\n", &l, &r);
                DisLeft = (double)l / UNIT;
                DisRight = (double)r / UNIT;
                DistanceDiff = DisLeft - DisRight;              //两轮行驶的距离差，m
                Distance = (DisLeft + DisRight) / 2.0;          //两轮平均行驶距离，m
                delta_theta = DistanceDiff / WHEEL_BASE;        //小车转向角，rad  当θ很小时，θ ≈ sin(θ)

                th += delta_theta;                              //朝向角，rad
                if (th > PI) th -= 2*PI;
                if (th < -PI) th += 2*PI;
                vth = delta_theta / REPOET_TIME;                //角速度，rad/s

                r = Distance / delta_theta;                     //转弯半径
                delta_x = delta_theta * r;
                delta_y = r * (1-cos(delta_theta));
                x += cos(th)*delta_x - sin(th)*delta_y;         //小车x坐标
                y += sin(th)*delta_x + cos(th)*delta_y;         //小车y坐标
                vx = (cos(th)*delta_x - sin(th)*delta_y) / REPOET_TIME;
                vy = (sin(th)*delta_x + cos(th)*delta_y) / REPOET_TIME;
                //ROS_INFO("acture   l=%d   r=%d   L=%lf   R=%lf   diff=%lf", l, r, DisLeft, DisRight, DistanceDiff);
                ROS_INFO("Odom Report:   x=%+7.4f y=%+7.4f th=%+7.4f   vx=%+7.4f vy=%+7.4f vth=%+7.4f", \
                    x, y, th, vx, vy, vth);

                ros::Time current_time = ros::Time::now();
                geometry_msgs::TransformStamped odom_trans;
                odom_trans.header.stamp = current_time;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_link";
                odom_trans.transform.translation.x = x;
                odom_trans.transform.translation.y = y;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);
                odom_broadcaster.sendTransform(odom_trans);

                nav_msgs::Odometry odom;
                odom.header.stamp = current_time;
                odom.header.frame_id = "odom";
                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);  //TODO
                odom.pose.pose.position.x = x;
                odom.pose.pose.position.y = y;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = odom_quat;
                odom.twist.twist.linear.x = vx;
                odom.twist.twist.linear.y = vy;
                odom.twist.twist.angular.z = vth;
                odom_pub.publish(odom);
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
        ros::Time current_time = ros::Time::now();
        geometry_msgs::PolygonStamped poly;
        geometry_msgs::Point32 point[4];
        point[0].x = 0;  point[0].y = -0.07;
        point[1].x = 0;   point[1].y = 0.07;
        point[2].x = -0.2;   point[2].y = 0.07;
        point[3].x = -0.2;  point[3].y = -0.07;
        poly.header.stamp = current_time;
        poly.header.frame_id = "base_link";
        poly.polygon.points.push_back(point[0]);
        poly.polygon.points.push_back(point[1]);
        poly.polygon.points.push_back(point[2]);
        poly.polygon.points.push_back(point[3]);
        poly_pub.publish(poly);

        r.sleep();
    }
}

int main(int argc, char** argv) {
    sp.reset(Stream::Serial("/dev/ttyTHS1", 230400));

    ros::init(argc, argv, "base_controller");
    ros::NodeHandle n;
    odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 10);
    poly_pub = n.advertise<geometry_msgs::PolygonStamped>("/polygon",10);
    vel_sub = n.subscribe("/cmd_vel", 10, CmdVelCallback);

    std::thread recv_task = std::thread(std::bind(SerialRecvTask));
    std::thread poly_task = std::thread(std::bind(PolyPubTask));

    ros::spin();

    if (recv_task.joinable()) {
        recv_task.join();
        ROS_INFO("recv_task_ joined!");
    }
    if (poly_task.joinable()) {
        poly_task.join();
        ROS_INFO("poly_task_ joined!");
    }
}
