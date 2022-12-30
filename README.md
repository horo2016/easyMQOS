# Introduce
easyMQOS is a simple and fast program to learn robot ，Distributed development framework for robots ，it supports c/c++ python  webjs .based on mqtt protocol.

easy_mqOS 是我仿照ROS 搭建的基于MQTT的简易机器人分布式开发框架,是一种轻量级并且十分容易上手的框架，支持多个节点的主题的订阅和单topic发布,节点之间独立、解耦合。没有复杂的文件配置，一定的make编程基础，像正常启动服务一样，就可以运行。甚至可以在嵌入式linux上使用，而不用安装Ubuntu没有复杂的插件，很容易上手和学习。支持c.c++ ,python,js.MQTT协议特点

使用发布/订阅消息模式，提供一对多的消息分发，解除应用程序耦合。
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
easymqos_waypoint_save   receive gps waypoints dats and save 接收规划路径的航点并保存   
### webjs的节点  
指南针  
轨迹航向显示  
twist 键盘控制      
baidu map 百度地图定位和路径规划

#  安装使用
首先在树莓派或者其他的Linux系统安装 mosquito   

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
