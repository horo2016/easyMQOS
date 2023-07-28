# Introduce
easyMQOS is a simple and fast program to learn robot ，Distributed development framework for robots ，it supports c/c++ python  webjs .based on mqtt protocol.

easy_mqOS 是我仿照ROS 搭建的基于MQTT的简易机器人分布式开发框架,是一种轻量级并且十分容易上手的框架，支持多个节点的主题的订阅和单topic发布,节点之间独立、解耦合。没有复杂的文件配置，一定的make编程基础，像正常启动服务一样，就可以运行。甚至可以在嵌入式linux上使用，而不用安装Ubuntu没有复杂的插件，很容易上手和学习。支持c.c++ ,python,js.MQTT协议特点

使用发布/订阅消息模式，提供一对多的消息分发，解除应用程序耦合。
# 公众号 视觉动力机器人
# 目的
打造低成本的室内slam建图车和导航车；  
打造户外GPS定位导航的自动驾驶车；   


# 说明
每个对应的节点下有对应的说明请仔细阅读  
### 本工程终端 的节点  
easymqOs_IMU_node   IMU的发布节点   
easymqOs_base_OdomControl  和底盘stm32通信的节点使用串口和自定义的通信协议 ，包头0xaa       
easymqOs_odom_imu_fusion   多传感器的融合航向角和位移   发布最终的位移和航向角度   
easymqOs_gpsKalmanfilter   gps filter node publish gps node   gps滤波节点并发布     
easymqOs_lidarA1_node    slamtech lidar a1 publish distance   发布雷达360度的距离    
easymqOs_opticalFlow_gl9306  GL9306光流传感器 串口接收 用于定位 调试中    
easymqos_waypoint_save   receive gps waypoints dats and save 接收规划路径的航点并保存     
easymqOs_collect_datas   收集里程计和雷达数据 用于离线建图    
easymqOs_detectDistance_vl53  收集 vl53l0的测距数据用于神经网络避障        
easymqOs_detectDistance_sonaras  收集 sonars超声波的测距数据用于神经网络避障      
easymqOs_AStar_planning    A star path planning ,规划路径并发布路径信息
### pytorch节点  
neural_network_avoidance  采集正前方5个角度的激光雷达测距消息 使用神经网络完成避障功能  
orangepizero2    
easymqOs_oledssd1306   采集bmm150 地磁计并显示oled   
easymqOs_navigation   截至2023/7/26 B站视频播放的使用该避障导航的程序    

### python 节点  
使用python发布节点和接收，支持发布图片  。
跑深度学习 摄像头实时目标检测，

### webjs的节点  
指南针  
轨迹航向显示  
twist 键盘控制      
baidu map 百度地图定位和路径规划
traject 显示雷达点云、图片流等  

#  安装使用
首先在树莓派或者其他的Linux系统安装 mosquito   

###  ubuntu  

sudo apt-get install mosquitto-dev   
sudo apt-get install mosquitto-clients   
然后在 /etc/mosquitto/mosquitto.conf 添加支持wensocket的协议和端口   
port 1883   
listener 8083  
protocol websockets  

下一步  
git clone this code   并查找libmosquitto.so 例如：   
'sunrise@ubuntu:~/$ sudo find / -name libmosquitto*   
[sudo] password for sunrise:    
/usr/lib/aarch64-linux-gnu/libmosquitto.so.1   
'   
将 mqtt的libmosquitto.so 拷贝到每个节点的Mqtt/lib下，         
cd  路径  
make   
分别执行对应的程序   
###  其他Linux系统   
源码编译 mosquitto   

#  建图效果   
![img](https://github.com/horo2016/easyMQOS/blob/main/img/1.png)   
![img](https://github.com/horo2016/easyMQOS/blob/main/img/712my1.png)     
![imng](https://github.com/horo2016/easyMQOS/blob/main/img/712my2.png)    
![img](https://github.com/horo2016/easyMQOS/blob/main/img/%E5%BE%AE%E4%BF%A1%E6%88%AA%E5%9B%BE_20230712173505.png)    
![img](https://github.com/horo2016/easyMQOS/blob/main/img/%E4%B8%AD%E5%80%BC%E6%BB%A4%E6%B3%A2.png)
