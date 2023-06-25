#include <ros/ros.h>
#include <cstdio>
#include <string>
#include <serial/serial.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

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
        ROS_ERROR("Unable to open camera1 port");
        return -1;
    }
 
    if (ser.isOpen()) {
        ROS_INFO("Camera1 port initialized");
        return 0;
    } else {
        return -1;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "camera1_node");
    ros::NodeHandle nh;
    //读取参数
    std::string serial_port;
    nh.getParam("camera1/serial_port", serial_port);


    //发布订阅
    //ros::Subscriber write_sub = nh.subscribe("serial/read", 200, write_callback);//订阅mcu串口数据
    ros::Publisher camera_pub = nh.advertise<std_msgs::Float32MultiArray>("camera1/read", 100);//发布camera串口数据
    
    if(open_serial(serial_port)==-1)
        return -1;
 
    //配置：
    //100hz频率执行
    std::string read_str;
    std_msgs::Float32MultiArray read_msg;
    float left;
    float right;
    float task;
    read_msg.data.resize(3);
    ros::Rate loop_rate(100);

    while(ros::ok){

        ros::spinOnce();

        if(ser.available()){
            read_str = ser.read(ser.available());
            //sscanf(read_str.c_str(), "%f %f %f", &left, &right, &task);
            //read_msg.data[0]=left;
            //read_msg.data[1]=right;
            //read_msg.data[2]=task;
            sscanf(read_str.c_str(), "%f %f %f", &left, &right, &task);
            read_msg.data[0]=left;
            read_msg.data[1]=right;
            read_msg.data[2]=task;
            ROS_INFO("%s",read_str.c_str());
            camera_pub.publish(read_msg);
            
        }
        loop_rate.sleep();
    }
    
    
    return 0;
}