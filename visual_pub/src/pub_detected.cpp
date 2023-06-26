#include <ros/ros.h>
#include <cstdio>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "visual_pub");
    ros::NodeHandle nh;

    ros::Publisher vis_pub = nh.advertise<std_msgs::Float32MultiArray>("camera1/read", 100);
    float first_crossing=0;
    float second_crossing=0;
    float third_crossing=0;
    float A1=0;
    float A2=0;
    float A3=0;
    int speed;
    nh.getParam("pub/first_crossing",first_crossing);
    nh.getParam("pub/second_crossing",second_crossing);
    nh.getParam("pub/third_crossing",third_crossing);
    nh.getParam("pub/A1",A1);
    nh.getParam("pub/A2",A2);
    nh.getParam("pub/A3",A3);
    nh.getParam("main_controller_pkg/default_speed",speed);
    first_crossing*=(speed/30);
    second_crossing*=(speed/30);
    third_crossing*=(speed/30);
    A1*=(speed/30);
    A2*=(speed/30);
    A3*=(speed/30);
    ROS_INFO("first_crossing:%0.2f,second_crossing:%0.2f,third_crossing:%0.2f,A1:%0.2f,A2:%0.2f,A3:%0.2f",first_crossing,second_crossing,third_crossing,A1,A2,A3);
    std_msgs::Float32MultiArray visual_data;
    double time_start;
    double time;
    int start=1;
    visual_data.data.resize(3);
    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        if(start==1){
            time_start=ros::Time::now().toSec();
            start=0;
        }
        time=ros::Time::now().toSec()-time_start;
        visual_data.data[0] = 10;
        visual_data.data[1] = 10;
        visual_data.data[2] = 0;

        
        if(first_crossing!=0&&time>first_crossing){
            visual_data.data[2] = 1.5;
            first_crossing=0;
            ROS_WARN("giving visual crossing1!");
        }
        else if(second_crossing!=0&&time>second_crossing){
            visual_data.data[2] = 1.5;
            second_crossing=0;
            ROS_WARN("giving visual crossing2!");
        }
        else if(third_crossing!=0&&time>third_crossing){
            visual_data.data[2] = 1.5;
            third_crossing=0;
            ROS_WARN("giving visual crossing3!");
        }
        else if(A1!=0&&time>A1){

            visual_data.data[2] = 2.5;
            A1=0;
            ROS_WARN("giving visual A1!");
        }
        else if(A2!=0&&time>A2){
            visual_data.data[2] = 2.5;
            A2=0;
            ROS_WARN("giving visual A2!");
        }
        else if(A3!=0&&time>A3){
            visual_data.data[2] = 2.5;
            A3=0;
            ROS_WARN("giving visual A3!");
        }
        vis_pub.publish(visual_data);

        loop_rate.sleep();
    }


}