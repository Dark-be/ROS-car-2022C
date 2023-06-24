#include <cstdio>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>



class Car{
public:
    int running;
    int inside;
    int following;
    float distance;
    int left_circle;
    std::string top_name;
    ros::Publisher vel_pub;
    std_msgs::Float32MultiArray vel_msg;
    ros::NodeHandle nh;
    int crossingdetected;
    int crossingdetectedlock;
    int Edetected;
    int Adetected;
    Car(const std::string& _top_name){
        running=0;//运行
        inside=0;//外圈
        following=0;//跟随
        left_circle=0;//剩余圈数
        top_name=_top_name;
        vel_pub = nh.advertise<std_msgs::Float32MultiArray>(_top_name, 200);
        vel_msg.data.resize(2);
        
    }
    void SetVel(float left,float right){
        vel_msg.data[0] = left;
        vel_msg.data[1] = right;
        vel_pub.publish(vel_msg);
        ROS_INFO("have set %0.2f,%0.2f",left,right);
    }
    void Turn(float r,float w){
        //(v1-v2)/10cm=w;
        //(v1+v2)/2=w*r;
        float left=w*r+5*w;//w>0左转
        float right=w*r-5*w;
        SetVel(left,right);
    }
    
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "main_control_node");

    Car car1("car1/write");

    int action_mode=0;
    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        
        switch(action_mode){
            case 0:{
                if(car1.running==0){
                    car1.Turn(10,0.5);
                }
                ROS_INFO("action:0 branch");
                break;
            }
            case 1:{
                ROS_INFO("action:1 branch");
                break;
            }
            default:{
                ROS_INFO("default branch");
                break;
            }
        }
        






        loop_rate.sleep();
    }


}