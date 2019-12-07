#include <math.h>
#include <iostream>
#include <thread>
#include "stream.h"

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// send the excepted number of pulses of the left&right wheel
// $+12345,+12345\n

// recv the actual number of pulses of the left&right wheel
// 55 AA E+00001,+00001\n
// recv the current battery voltage
// 55 AA B+01234\n
// recv gyro, acc, quaternion
// 55 AA I%f%f%f%f%f%f%f%f%f%f\n

#define WHEEL_BASE (0.178)         //轮距 m
#define PERIMITER (0.238)          //轮子周长 m
#define UNIT (1040.0 / PERIMITER)  //每米对应的编码器脉冲数
#define CONTROL_TIME (0.05)        //编码器采样周期 5ms
#define REPORT_TIME (0.05)         //小车里程计上报周期
#define PI (3.14159265358979)

using namespace std;
std::shared_ptr<Stream> sp;
char send_buf[32];
char recv_buf[64];

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
  int desire_left = linear_speed * UNIT * CONTROL_TIME;
  int desire_right = linear_speed * UNIT * CONTROL_TIME;
  // 角速度 rad/s
  // 本次要转动的角度 theta = DesireAngVelo * CONTROL_TIME
  // Theta = dis / base   dis = theta * base
  // 每个轮子移动距离 d = dis/2 = theta * base / 2
  int diff_distance = angular_speed * CONTROL_TIME * WHEEL_BASE / 2.0 * UNIT;
  desire_left += diff_distance;
  desire_right -= diff_distance;
  memset(send_buf, 0, sizeof(send_buf));
  sprintf(send_buf, "$%+06d,%+06d\n", desire_left, desire_right);
  ROS_INFO("desire   L=%d   R=%d   diff: %d", desire_left, desire_right,
           diff_distance);
  sp->Write(reinterpret_cast<uint8_t *>(send_buf), strlen(send_buf));
  ROS_INFO("send: %s", send_buf);
}

int RecvReport(char *buf) {
  int retry_times = 100, ret = 0;
  char sync = 0;
  while (retry_times--) {
    ret = sp->Read(reinterpret_cast<uint8_t *>(&sync), 1, 1);
    if (ret != 1 || sync != '~') {
      continue;
    }
    // ROS_INFO("sync1 %02X...", sync);
    ret = sp->Read(reinterpret_cast<uint8_t *>(&sync), 1, 1);
    if (ret != 1 || sync != '>') {
      continue;
    }
    // ROS_INFO("sync2 %02X...", sync);
    ret = sp->Read(reinterpret_cast<uint8_t *>(buf), 1, 1);
    if (ret != 1 || (buf[0] != 'E' && buf[0] != 'B' && buf[0] != 'I')) {
      continue;
    }
    // ROS_INFO("recv type %c...", buf[0]);
    if (buf[0] == 'E') {
      ret = sp->Read(reinterpret_cast<uint8_t *>(buf + 1), 14, 64);
    } else if (buf[0] == 'B') {
      ret = sp->Read(reinterpret_cast<uint8_t *>(buf + 1), 7, 64);
    } else if (buf[0] == 'I') {
      ret = sp->Read(reinterpret_cast<uint8_t *>(buf + 1), 41, 64);
    }
    // ROS_INFO("%s", buf);
    return 0;
  }
  return -1;
}

void SerialRecvTask() {
  int l = 0, r = 0;
  int voltage = 0;
  float gyro[3], acc[3], q[4];

  double distance_left = 0, distance_right = 0, distance = 0, diff_distance = 0;
  double delta_x, delta_y, delta_theta = 0, radius = 0;
  double x = 0, y = 0, th = 0;
  double vx = 0, vy = 0, vth = 0;
  ros::Time current_time;
  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom;
  geometry_msgs::Quaternion odom_quat;
  tf::TransformBroadcaster odom_broadcaster;

  while (ros::ok()) {
    memset(recv_buf, 0, sizeof(recv_buf));
    if (RecvReport(recv_buf)) continue;

    switch (recv_buf[0]) {
      case 'E':
        sscanf(recv_buf, "E%d,%d\n", &l, &r);
        if (l || r) {
          distance_left = (double)l / UNIT;
          distance_right = (double)r / UNIT;
          //两轮平均行驶距离
          distance = (distance_left + distance_right) / 2.0;
          //两轮行驶的距离差
          diff_distance = distance_left - distance_right;
          if (diff_distance) {
            //小车转向角，rad  当θ很小时，θ ≈ sin(θ)
            delta_theta = diff_distance / WHEEL_BASE;
            //朝向角，rad
            th += delta_theta;
            if (th > PI) th -= 2 * PI;
            if (th < -PI) th += 2 * PI;
            //角速度，rad/s
            vth = delta_theta / REPORT_TIME;
            //转弯半径
            radius = distance / delta_theta;
            delta_x = delta_theta * radius;
            delta_y = radius * (1 - cos(delta_theta));
            //小车x坐标
            x = x + cos(th) * delta_x - sin(th) * delta_y;
            //小车y坐标
            y = y + sin(th) * delta_x + cos(th) * delta_y;
            vx = (cos(th) * delta_x - sin(th) * delta_y) / REPORT_TIME;
            vy = (sin(th) * delta_x + cos(th) * delta_y) / REPORT_TIME;
          } else {
            x += distance_left;
            y = y;
            th = th;
            vx = distance_left / REPORT_TIME;
            vy = 0;
            vth = 0;
          }
        } else {
          vx = 0;
          vy = 0;
          vth = 0;
        }
        // ROS_INFO("acture   l=%d   r=%d   L=%lf   R=%lf   diff=%lf", l, r,
        // distance_left, distance_right, diff_distance);
        ROS_INFO("Odom Report: x=%+7.4lf y=%+7.4lf th=%+7.4lf vx=%+7.4lf "
                 "vy=%+7.4lf vth=%+7.4f", x, y, th, vx, vy, vth);

        current_time = ros::Time::now();
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);
        odom_broadcaster.sendTransform(odom_trans);

        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, th);  // TODO
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        odom_pub.publish(odom);
        break;
      case 'B':
        sscanf(recv_buf, "B%d\n", &voltage);
        ROS_INFO("Voltage Report: %.2fV\n", voltage / 100.0);
        break;
      case 'I':
        gyro[0] = *(float *)(recv_buf + 1 + 0);
        gyro[1] = *(float *)(recv_buf + 1 + 4);
        gyro[2] = *(float *)(recv_buf + 1 + 8);
        acc[0] = *(float *)(recv_buf + 1 + 12 + 0);
        acc[1] = *(float *)(recv_buf + 1 + 12 + 4);
        acc[2] = *(float *)(recv_buf + 1 + 12 + 8);
        q[0] = *(float *)(recv_buf + 1 + 24 + 0);
        q[1] = *(float *)(recv_buf + 1 + 24 + 4);
        q[2] = *(float *)(recv_buf + 1 + 24 + 8);
        q[3] = *(float *)(recv_buf + 1 + 24 + 12);
        // sscanf(recv_buf+1, "%f%f%f%f%f%f%f%f%f%f",
        //    &gyro[0], &gyro[1], &gyro[2],
        //    &acc[0], &acc[1], &acc[2],
        //    &q[0], &q[1], &q[2], &q[3]);
        ROS_INFO("%+7.4lf %+7.4lf %+7.4lf\t%+7.4lf %+7.4lf %+7.4lf\t%+7.4lf %+7.4lf %+7.4lf %+7.4lf", gyro[0], gyro[1], gyro[2],
                 acc[0], acc[1], acc[2], q[0], q[1], q[2], q[3]);
        break;
      default:
        ROS_INFO("Check failed, recv: %s", recv_buf);
        break;
    }
  }
}

void PolyPubTask() {
  ros::Rate r(10);

  while (ros::ok()) {
    ros::Time current_time = ros::Time::now();
    geometry_msgs::PolygonStamped poly;
    geometry_msgs::Point32 point[4];
    point[0].x = 0;
    point[0].y = -0.07;
    point[1].x = 0;
    point[1].y = 0.07;
    point[2].x = -0.2;
    point[2].y = 0.07;
    point[3].x = -0.2;
    point[3].y = -0.07;
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

int main(int argc, char **argv) {
  sp.reset(Stream::Serial("/dev/ttyTHS1", 230400));

  ros::init(argc, argv, "base_controller");
  ros::NodeHandle n;
  odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 10);
  poly_pub = n.advertise<geometry_msgs::PolygonStamped>("/polygon", 10);
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
