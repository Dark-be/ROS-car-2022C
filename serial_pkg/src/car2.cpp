#include <ros/ros.h>
#include <cstdio>
#include <string>
#include <serial/serial.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <signal.h>

int x_z=0;//是否x_z类型速度
// 从机车需发布camera2/read话题
//运行于主机，从机发送camera2/read数据，本节点发布camera2/read话题，相当于结合了car1和camera1两个节点
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
const uint8_t head[3]={0xC8,0xFF,0x00};
float data[2];
//浮点转字节码并发送字节码，通知速度
void write_callback(const std_msgs::Float32MultiArray& msg){
    if(x_z){
        data[0] = msg.data[0]-msg.data[1]*6;
        data[1] = msg.data[0]+msg.data[1]*6;
    }
    else {
        data[0] = msg.data[0];
        data[1] = msg.data[1];
    }

    uint8_t* buffer = reinterpret_cast<uint8_t*>(&data[0]);
    ROS_INFO("Have set car2 vel{left:%0.2f,rigth:%0.2f}",msg.data[0],msg.data[1]);
    ser.write(head, 3);
    ser.write(buffer, 8);
}
void signalHandler(int sig)
{
    // 在这里编写你的中断处理代码，中断停车
    const uint8_t head[3]={0xC8,0xFF,0x00};
    float data[2]={0,0};
    uint8_t* buffer = reinterpret_cast<uint8_t*>(&data[0]);
    ser.write(head, 3);
    ser.write(buffer, 8);
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "car2_node");
    ros::NodeHandle nh;
    //读取参数
    std::string serial_port;
    nh.getParam("car2/serial_port", serial_port);
    nh.getParam("car2/x_z", x_z);
    //发布订阅
    ros::Subscriber write_sub = nh.subscribe("car2/write", 100, write_callback);//写串口话题
    ros::Publisher camera_pub = nh.advertise<std_msgs::Float32MultiArray>("camera2/read", 100);//读串口摄像头话题
    signal(SIGINT, signalHandler);

    if(open_serial(serial_port)==-1)
        return -1;
    
    // 100hz频率执行
    std::string read_str;
    std_msgs::Float32MultiArray camera_msg;
    float left;
    float right;
    float task;
    ros::Rate loop_rate(100);

    while (ros::ok()) {
        ros::spinOnce();
        if (ser.available()) {
            read_str=ser.read(ser.available());
            sscanf(read_str.c_str(), "%f %f %f", &left, &right, &task);
            camera_msg.data[0]=left;
            camera_msg.data[1]=right;
            camera_msg.data[2]=task;
            ROS_INFO("camera2:%s",read_str.c_str());
            camera_pub.publish(camera_msg);
        }
        loop_rate.sleep();
    }
}