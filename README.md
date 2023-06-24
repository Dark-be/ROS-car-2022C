# ROS-car-2022C

ros实现2022C题(TI)跟随小车

## Instructions
1.修改main_controller_pkg中的control.launch对应串口参数  
2.运行control.launch  

## Introduce
使用jetson nano作为上位机  
摄像头具体话题接口：camera*/read  
Float32MultiArray:  
data[0]:left  
data[1]:right  
data[2]:task  
下位机具体话题接口car*/write  
Float32MultiArray:  
data[0]:left  
data[1]:right  
均接收真实速度(cm/s)  

