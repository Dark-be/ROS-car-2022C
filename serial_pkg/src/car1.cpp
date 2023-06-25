#include <ros/ros.h>
#include <cstdio>
#include <string>
#include <serial/serial.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include <tf/transform_datatypes.h>
#include <signal.h>

//串口类
serial::Serial ser;
//开串口
int open_serial(std::string port){
    try {
        ser.setPort(port);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException &e) {
        ROS_ERROR("Unable to open car1 port %s", port.c_str());
        return -1;
    }
 
    if (ser.isOpen()) {
        ROS_INFO("Car1 Port %s initialized", port.c_str());
        return 0;
    } else {
        return -1;
    }
}
//--------------------------------------------------

//浮点转字节码并发送字节码，通知速度
void write_callback(const std_msgs::Float32MultiArray& msg){
    const uint8_t head[3]={0xC8,0xFF,0x00};
    float data[2];
    data[0] = msg.data[0];
    data[1] = msg.data[1];
    uint8_t* buffer = reinterpret_cast<uint8_t*>(&data[0]);
    ROS_INFO("Have set car1 vel{left:%0.2f,rigth:%0.2f}",msg.data[0],msg.data[1]);
    ser.write(head, 3);
    ser.write(buffer, 8);
}
void signalHandler(int sig)
{
    // 在这里编写你的中断处理代码
    const uint8_t head[3]={0xC8,0xFF,0x00};
    float data[2]={0,0};
    uint8_t* buffer = reinterpret_cast<uint8_t*>(&data[0]);
    ser.write(head, 3);
    ser.write(buffer, 8);
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "car1_node");
    ros::NodeHandle nh;
    //读取参数
    std::string serial_port;
    nh.getParam("car1/serial_port", serial_port);
    //发布订阅
    ros::Subscriber write_sub = nh.subscribe("car1/write", 100, write_callback);//写串口话题
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("car1/read", 100);//读串口话题
    ros::Publisher IMU_pub = nh.advertise<sensor_msgs::Imu>("serial/imu", 200);//发布imu数据
    //ros::Publisher Odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 200);//发布里程计数据
    ros::Publisher Twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("car1/twist", 100);//发布速度数据
    //ros::Publisher Pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("car1/pose", 100);//发布位姿数据
    signal(SIGINT, signalHandler);


    if(open_serial(serial_port)==-1)
        return -1;
    
    // 100hz频率执行
    std_msgs::String read_msg;
    sensor_msgs::Imu imu_data;
    //geometry_msgs::TwistWithCovarianceStamped current_twist;
    //geometry_msgs::PoseWithCovarianceStamped current_pose;

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        //current_twist.header.seq = frame_sequence_;
        //current_twist.header.stamp = ros::Time::now();
        //current_twist.header.frame_id = "base_link";
        //current_twist.twist.twist.linear.x = 0.;
        //current_twist.twist.twist.linear.y = 0.;
        //current_twist.twist.twist.linear.z = 0.;
        //current_twist.twist.twist.angular.x = 0.;
        //current_twist.twist.twist.angular.y = 0.;
        //current_twist.twist.twist.angular.z = 1.;
        //Twist_pub.publish(current_twist);

        ////位置信息
        //current_pose.header.seq = frame_sequence_;
        //current_pose.header.stamp = ros::Time::now();
        //current_pose.header.frame_id = "map";
        //current_pose.pose.pose.position.x = 0.;
        //current_pose.pose.pose.position.y = 0.;
        //current_pose.pose.pose.position.z = 0.;
        //current_pose.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0., 0., frame_sequence_/100 *2);
        ////current_pose.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0., 0., 0.);
        //Pose_pub.publish(current_pose);

        //imu_data.header.stamp = ros::Time::now();
        //imu_data.header.frame_id = "base_link";
        ////四元数位姿,所有数据设为固定值，可以自己写代码获取IMU的数据，，然后进行传递
        //imu_data.orientation.x = 0;
        //imu_data.orientation.y = 0;
        //imu_data.orientation.z = 0;
        //imu_data.orientation.w = 1;
        ////线加速度
        //imu_data.linear_acceleration.x = 0.0; 
        //imu_data.linear_acceleration.y = 0.0;
        //imu_data.linear_acceleration.z = 0.0;
        ////角速度
        //imu_data.angular_velocity.x = 0.00; 
        //imu_data.angular_velocity.y = 0.00; 
        //imu_data.angular_velocity.z = 0.00;
        //IMU_pub.publish(imu_data);
        if (ser.available()) {
            read_msg.data=ser.read(ser.available());
            read_pub.publish(read_msg);

            //imu_data.header.stamp = ros::Time::now();
            //imu_data.header.frame_id = "base_link";
            ////四元数位姿,所有数据设为固定值，可以自己写代码获取IMU的数据，，然后进行传递
            //imu_data.orientation.x = 0;
            //imu_data.orientation.y = 0;
            //imu_data.orientation.z = 0;
            //imu_data.orientation.w = 1;
            ////线加速度
            //imu_data.linear_acceleration.x = 0.01; 
            //imu_data.linear_acceleration.y = 0.02;
            //imu_data.linear_acceleration.z = 0.03;
            ////角速度
            //imu_data.angular_velocity.x = 0.00; 
            //imu_data.angular_velocity.y = 0.00; 
            //imu_data.angular_velocity.z = 0.00;
            //IMU_pub.publish(imu_data);
            //odom_data.header.stamp = ros::Time::now();
            //odom_data.header.frame_id = "odom";
            
        }
        loop_rate.sleep();
    }
}