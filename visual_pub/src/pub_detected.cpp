#include <ros/ros.h>
#include <cstdio>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "visual_pub");
    ros::NodeHandle nh;
    float first_crossing=0;
    float second_crossing=0;
    float third_crossing=0;
    nh.getParam("first_crossing",first_crossing);
    nh.getParam("second_crossing",second_crossing);
    nh.getParam("third_crossing",third_crossing);

    std_msgs::Float32MultiArray visual_data;
    float time;
    visual_data.data.resize(3);
    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();

        visual_data.data[0] = 10;
        visual_data.data[1] = 10;
        visual_data.data[2] = 0;

        time=ros::Time::now().toSec();
        if((first_crossing!=0&&time>first_crossing)||(second_crossing!=0&&time>second_crossing)||(third_crossing!=0&&time>third_crossing)){
            visual_data.data[0] = 10;
            visual_data.data[1] = 10;
            visual_data.data[2] = 1.5;
        }

        loop_rate.sleep();
    }


}