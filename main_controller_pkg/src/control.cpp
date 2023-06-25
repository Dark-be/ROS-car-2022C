#include <cstdio>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>
#include <signal.h>
#include "my_pid.h"

#define MAX_TURN 25.0


//---------------------------------------------------------------------------------回调函数区
int default_speed_p=0;
int trackline_p=1;
int test_p=0;
int distance_p=10;
int target_distance_p=20;
//行为模式 1~4 0为待输入
//行为模式订阅//暂时使用launch文件修改
int mode=0;
int mode_lock=0;
void mode_callback(std_msgs::Int32 msg){
    if(msg.data>0&&msg.data<5&&!mode_lock){
        mode=msg.data;
    }
}

//状态订阅
typedef struct camera_state
{
    float tright=0;
    float tleft=0;
    int crossing_detected=0;
    int Adetected=0;
    int Edetected=0;
}camera_state;

//距离订阅
float front_distance=0;
float back_distance=0;
void scan_callback(std_msgs::Float32MultiArray msg){
    front_distance=msg.data[0]*100;
    back_distance=msg.data[1]*100;
    ROS_INFO("distance:%f %f",front_distance,back_distance);
}

PID distance_pid(5,0,0,0,30);

class Car{
public:
    //基本状态
    int running;                        //运行
    float expect_speed;                 //期望速度
    float default_speed;                //默认速度
    int following;                      //是否跟随
    int left_circle;                    //剩余圈数

    int state;
    int position;

    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Subscriber cam_sub;
    std_msgs::Float32MultiArray vel_msg;

    //状态回调
    camera_state cam_state;//相机状态
    float distance;//距离状态
    
    //固定动作
    int action;
    double time;
    double last_time;
    int time_lock;

    void cam_callback(std_msgs::Float32MultiArray msg){
        cam_state.tright=msg.data[0]-10;
        cam_state.tleft=msg.data[1]-10;
        if(cam_state.tleft>MAX_TURN)
            cam_state.tleft=MAX_TURN;
        else if(cam_state.tleft<-MAX_TURN)
            cam_state.tleft=-MAX_TURN;
        if(cam_state.tright>MAX_TURN)
            cam_state.tright=MAX_TURN;
        else if(cam_state.tright<-MAX_TURN)
            cam_state.tright=-MAX_TURN;
        ROS_INFO("t_left:%f t_right:%f",cam_state.tleft,cam_state.tright);
        cam_state.crossing_detected=0;
        cam_state.Adetected=0;
        cam_state.Edetected=0;
        if(1<msg.data[2]&&msg.data[2]<2){//检测路口置高
            cam_state.crossing_detected=1;
            ROS_INFO("crossing");
        }
        else if(2<msg.data[2]&&msg.data[2]<3){//检测A置高
            cam_state.Adetected=1;
            ROS_INFO("A");
        }
        else if(3<msg.data[2]&&msg.data[2]<4){//检测E置高
            cam_state.Edetected=1;
            ROS_INFO("E");
        }
    }
    Car(const std::string& pub_name,const std::string& sub_name){
        running=0;
        expect_speed=0;
        default_speed=0;
        following=0;
        left_circle=0;
        state=0;
        position=0;

        cam_state={0,0,0,0,0};

        
        time_lock=0;
        action=0;

        vel_pub = nh.advertise<std_msgs::Float32MultiArray>(pub_name, 100);
        cam_sub = nh.subscribe(sub_name, 100, &Car::cam_callback,this);
        
        vel_msg.data.resize(2);
    }
    void SetVel(float left,float right){
        vel_msg.data[0] = left;
        vel_msg.data[1] = right;
        vel_pub.publish(vel_msg);
    }
    void Straight(){
        SetVel(expect_speed,expect_speed);
    }
    void Turn(float r){
        //(v1-v2)/10cm=w;
        //(v1+v2)/2=w*r;
        float w=expect_speed/r;
        float left=expect_speed+15*w;
        float right=expect_speed-15*w;
        SetVel(left,right);
        
    }
    void StraightFor(float _last_time){
        if(time_lock==0){
            last_time=_last_time;
            time=ros::Time::now().toSec();
            action=1;
            time_lock=1;
            ROS_INFO("straighting");
        }
        
    }
    void TurnFor(float _last_time){
        if(time_lock==0){
            last_time=_last_time;
            time=ros::Time::now().toSec();
            action=2;
            time_lock=1;
            ROS_INFO("turnning");
        }
        
    }
    void DoingLock(){
        if(test_p==1)
            time_lock=0;
        if(ros::Time::now().toSec()-time<last_time){
            if(action==1){

                Straight();
            }
            else if(action==2){

                Turn(-30);
            }
                
        }
        else if(ros::Time::now().toSec()-time>=last_time){
            ROS_INFO("Time unlock");
            time_lock=0;
        }
    }
    void TrackLine(float _left,float _right){
        SetVel(expect_speed+_left,expect_speed+_right);
    }
    void Stop(){
        SetVel(0,0);
    }
    
    //状态机------------------------------------------------------------------------------------
    //car行为判断
    void UpdateRunning(){
        
        if(time_lock==1){//锁住做固定动作
            if(left_circle>0)
                DoingLock();
        }
        if(running==1&&time_lock==0){//car运行中且未锁
            if(left_circle>0||test_p==1){
                if(following==1){//跟随模式开启，仅改变期望速度，仍然巡线以及固定动作
                    if(distance>(target_distance_p-distance_p)&&distance<(target_distance_p+distance_p)){
                        expect_speed-=distance_pid.GetRes(distance,target_distance_p);
                    }
                }
                if(trackline_p==1)
                    TrackLine(cam_state.tleft,cam_state.tright);
                else
                    Straight();
            }
            else{//剩余圈数为0，停止
                Stop();
            }
        }
    }
    void UpdatePosition(int loop){//位置信号
        position++;
        if(position>=loop){
            position=0;
        }
    }
    void UpdateCamState(){

    }
    //路线总共两段，A->crossing crossing->A 对应 00->(01->00)->(10-00)状态转换函数
    //position对应0，1(已测试，逻辑ok)
    void car1_car2_UpdateStateOf12(){
        state=cam_state.Adetected*10+cam_state.crossing_detected;
        if(state==/*0*/1){//检测到路口并处于对应路段
            if(position==0){
                StraightFor(30/expect_speed);
                cam_state.crossing_detected=0;
                UpdatePosition(2);
            }
        }
        else if(state==10){//检测到A点并处于对应路段
            if(position==1){
                left_circle--;
                cam_state.Adetected=0;
                UpdatePosition(2);
            }
        }
        expect_speed=default_speed;
    }

    void test_UpdateStateOf12(){
        state=cam_state.Adetected*10+cam_state.crossing_detected;
        if(state==/*0*/1){//检测到路口并处于对应路段
            if(position==0){
                TurnFor(30/expect_speed);
                cam_state.crossing_detected=0;
                UpdatePosition(2);
            }
        }
        else if(state==10){//检测到A点并处于对应路段
            if(position==1){
                left_circle--;
                cam_state.Adetected=0;
                UpdatePosition(2);
            }
        }
        if(mode==1)
            expect_speed=30;
        else if(mode==2)
            expect_speed=30;
        expect_speed=default_speed;
    }
    //路线总共三段，A->crosing crossing->A 对应 00->(01->00)->(10-00)状态转换函数
    //position对应0，1
    void car1_UpdateStateOf3(){//car1为外外内 领跟领
        state=cam_state.Adetected*10+cam_state.crossing_detected;
        if(state==/*0*/1){//检测到路口并处于对应路段
            if(position==0){
                if(left_circle==3){
                    StraightFor(30/expect_speed);
                }
                else if(left_circle==2){
                    following=1;
                    StraightFor(30/expect_speed);
                }
                else if(left_circle==1){
                    following=0;
                    TurnFor(30*0.7/expect_speed);
                }
                cam_state.crossing_detected=0;
                UpdatePosition(2);
            }
        }
        else if(state==10){//检测到A点并处于对应路段
            if(position==1){
                left_circle--;
                cam_state.Adetected=0;
                UpdatePosition(2);
            }
        }
        expect_speed=default_speed;
    }
    void car2_UpdateStateOf3(){
        state=cam_state.Adetected*10+cam_state.crossing_detected;
        if(state==/*0*/1){//检测到路口并处于对应路段
            if(position==0){
                if(left_circle==3){
                    StraightFor(30/expect_speed);
                }
                else if(left_circle==2){
                    following=0;
                    TurnFor(30*0.7/expect_speed);
                }
                else if(left_circle==1){
                    following=1;
                    StraightFor(30/expect_speed);
                }
                cam_state.crossing_detected=0;
                UpdatePosition(2);
            }
        }
        else if(state==10){//检测到A点并处于对应路段
            if(position==1){
                left_circle--;
                cam_state.Adetected=0;
                UpdatePosition(2);
            }
        }
        expect_speed=default_speed;
    }
    void car1_car2_UpdateStateOf4(){//car1 car2 逻辑一样，初始不同 001  010 
        state=cam_state.Adetected*100+cam_state.Edetected*10+cam_state.crossing_detected;
        if(state==1){//检测分叉路口并处于对应路段
            if(position==0){
                cam_state.crossing_detected=0;
                StraightFor(30/expect_speed);
                UpdatePosition(3);
            }
        }
        else if(state==10){//检测到E点并处于对应路段
            if(position==1){
                left_circle--;
                cam_state.Edetected=0;
                UpdatePosition(3);
            }
        }
        expect_speed=default_speed;
    }
    //-------------------------------------------------------------------------------------------
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "main_control1_node");
    ros::NodeHandle nh;
    Car car1("car1/write","camera1/read");
    Car car2("car2/write","camera2/read");

    //读取行动方案(后续改为订阅)
    nh.getParam("main_control1/mode",mode);
    nh.getParam("main_control1/default_speed",default_speed_p);
    nh.getParam("main_control1/trackline",trackline_p);
    nh.getParam("main_control1/test",test_p);
    nh.getParam("main_control1/distance",distance_p);
    nh.getParam("main_control1/target_distance",target_distance_p);
    
    ros::Subscriber mode_sub = nh.subscribe("mode", 100, mode_callback);
    ros::Subscriber distance_sub = nh.subscribe("scan1/read", 100, scan_callback);

    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        if(mode_lock==0){//每次上电后只执行一次，传入mode后即锁定，对于每个行动方案，初始化小车
            mode_lock=1;
            switch(mode){
                case 1:{
                    car1.running=1;
                    car1.left_circle=1;//剩余圈数
                    car1.default_speed=default_speed_p;//期望速度
                    car1.state=0;//初状态
                    car1.following=0;//关闭跟随模式

                    car2.running=1;
                    car2.left_circle=1;
                    car2.default_speed=default_speed_p;
                    car2.state=0;
                    car2.following=1;

                    break;
                }
                case 2:{
                    car1.running=1;
                    car1.left_circle=2;
                    car1.default_speed=default_speed_p;
                    car1.state=0;
                    car1.following=1;

                    car2.running=1;
                    car2.left_circle=1;
                    car2.default_speed=default_speed_p;
                    car2.state=0;
                    car2.following=1;
                    break;
                }
                case 3:{
                    car1.running=1;
                    car1.left_circle=3;
                    car1.default_speed=default_speed_p;
                    car1.state=0;
                    car1.following=0;
                    

                    break;
                }
                case 4:{
                    car1.running=1;
                    car1.left_circle=1;
                    car1.default_speed=default_speed_p;
                    car1.state=0;
                    car1.following=0;

                    break;
                }
                case 5:{
                    car1.running=1;
                    car1.left_circle=1;
                    car1.default_speed=default_speed_p;
                    car1.state=0;
                    car1.following=0;



                    break;
                }
                default:{
                    break;
                }  
            }
            ROS_INFO("action %d",mode);
            ROS_INFO("config: mode=%d, default_speed=%d, trackline=%d, test=%d, distance=%d, target_distance=%d",mode,default_speed_p,trackline_p,test_p,distance_p,target_distance_p);
        }
        else {//已完成初始化，根据行动方案执行，未完成则进入default
            switch(mode){
                //方案1：car2跟随car1，car1跟随线路
                //car1：跟随线路
                //car2：跟随car1，跟随线路
                case 1:{

                    car1.distance=front_distance;
                    car1.car1_car2_UpdateStateOf12();//更新状态，圈数，跟随模式，时间锁等
                    car1.UpdateRunning();//更新对应行为

                    car2.distance=front_distance;
                    car2.car1_car2_UpdateStateOf12();
                    car2.UpdateRunning();
                    break;
                }
                //方案2：
                //
                case 2:{

                    car1.distance=front_distance;
                    car1.car1_car2_UpdateStateOf12();
                    car1.UpdateRunning();

                    car2.distance=front_distance;
                    car2.car1_car2_UpdateStateOf12();
                    car2.UpdateRunning();

                    break;
                }
                //方案3：
                //car1：跟随线路，insid时序001
                //car2：跟随car1，跟随线路，insid时序010
                case 3:{

                    car1.distance=front_distance;
                    car1.car1_UpdateStateOf3();
                    car1.UpdateRunning();

                    car2.distance=front_distance;
                    car2.car1_car2_UpdateStateOf12();
                    car2.UpdateRunning();


                    break;
                }
                case 4:{

                    car1.distance=front_distance;
                    car1.car1_car2_UpdateStateOf4();
                    car1.UpdateRunning();//更新对应行为





                    break;
                }
                case 5:{

                    car1.distance=front_distance;
                    car1.test_UpdateStateOf12();
                    car1.UpdateRunning();//更新对应行为
                    break;
                }
                default:{
                    car1.Stop();
                    ROS_INFO("wait for mode");
                    break;
                }
            }
            ROS_INFO("car1:state:%d,position:%d,left_circle:%d,distance:%0.2f,expect_speed:%0.2f,following:%d,time_lock:%d,action:%d",car1.state,car1.position,car1.left_circle,car1.distance,car1.expect_speed,car1.following,car1.time_lock,car1.action);
            ROS_INFO("car2:state:%d,position:%d,left_circle:%d,distance:%0.2f,expect_speed:%0.2f,following:%d,time_lock:%d,action:%d",car2.state,car2.position,car2.left_circle,car2.distance,car2.expect_speed,car2.following,car2.time_lock,car2.action);
        }   
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}
