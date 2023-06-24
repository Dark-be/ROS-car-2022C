# ROS-car-2022C
ros实现2022C题(TI)跟随小车

使用jetson nano作为上位机，main_controller_pkg中的control.launch作为启动文件。
运行需修改对应串口参数。
摄像头使用K210，具体话题接口：camera*/read
Float32MultiArray:
data[0]:left
data[1]:right
data[2]:task
下位机暂定，接收真实速度(cm/s)，具体话题接口car*/write
Float32MultiArray:
data[0]:left
data[1]:right
