# ROS-car-2022C

ros实现2022C题(TI)跟随小车

## 运行过程
1. 修改main_controller_pkg中的control.launch对应串口参数  
2. 运行control.launch  

## 介绍
1. main_controller_pkg包括了主要状态机转换，以及逻辑控制  
2. serial_pkg实现了摄像头，小车的串口通信，通过topic实现通信  
3. visual_pub实现了虚拟发布，可编写代码模拟外部话题发布，便于调试逻辑  
<br>
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

