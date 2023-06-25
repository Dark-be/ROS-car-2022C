#include <ros/ros.h>
#include <cstdio>
#include <string>
#include <serial/serial.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

// 从机车需发布camera2/read话题
//运行于从机，从机发送camera2数据
//串口类
serial::Serial sercar;//发
serial::Serial sercam;//收
serial::Serial serblue;//发
//开串口
int open_serial(serial::Serial& ser,std::string port){
    try {
        ser.setPort(port);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException &e) {
        ROS_ERROR("Unable to open car2 port %s", port.c_str());
        return -1;
    }
 
    if (ser.isOpen()) {
        ROS_INFO("Car2 Port %s initialized", port.c_str());
        return 0;
    } else {
        return -1;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "slave_node");
    ros::NodeHandle nh;
    //读取参数
    std::string car_port;
    std::string camera_port;
    std::string blue_port;
    nh.getParam("slave/car_port", car_port);//下位机端口
    nh.getParam("slave/camera_port", camera_port);//摄像头端口
    nh.getParam("slave/blue_port", blue_port);//蓝牙端口
    //发布订阅
    ros::Publisher camera_pub = nh.advertise<std_msgs::Float32MultiArray>("camera2/read", 100);//读串口摄像头话题

    open_serial(sercar,car_port);
    open_serial(sercam,camera_port);
    open_serial(serblue,blue_port);
    std::string read_str;
    // 100hz频率执行
    ros::Rate loop_rate(100);

    while (ros::ok()) {
        ros::spinOnce();
        if (sercar.available()) {
            read_str=sercar.read(sercar.available());
            ROS_INFO("recv:%s",read_str.c_str());
            sercar.write(reinterpret_cast<const uint8_t*>(read_str.c_str()), read_str.length());//传递给下位机
        }
        if(sercam.available()){
            read_str=sercam.read(sercam.available());
            ROS_INFO("recv:%s",read_str.c_str());
            serblue.write(reinterpret_cast<const uint8_t*>(read_str.c_str()), read_str.length());//传递给蓝牙
        }
        loop_rate.sleep();
    }
}