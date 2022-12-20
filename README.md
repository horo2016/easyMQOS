# easyMQOS is a simple and fast program to learn robot ，Distributed development framework for robots ，it supports c/c++ python  webjs .based on mqtt protocol.

easy_mqOS 是我仿照ROS 搭建的基于MQTT的简易机器人分布式开发框架,是一种轻量级并且十分容易上手的框架，支持多个节点的主题的订阅和单topic发布,节点之间独立、解耦合。没有复杂的文件配置，一定的make编程基础，像正常启动服务一样，就可以运行。甚至可以在嵌入式linux上使用，而不用安装Ubuntu没有复杂的插件，很容易上手和学习。支持c.c++ ,python,js.MQTT协议特点

使用发布/订阅消息模式，提供一对多的消息分发，解除应用程序耦合。

# 说明
# 本工程包括 多个节点
# easymqOs_IMU_node   IMU的发布节点
# easymqOs_base_OdomControl  和底盘stm32通信的节点使用串口和自定义的通信协议 ，包头0xaa 
