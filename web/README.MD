#  说明
该程序配合mqtt使用 ；  
js 需要结合本包中的paho-mqtt 一起使用。
在树莓派或者其他派终端安装mqtt 工具包，web可以使用boa 容器。  

首先在树莓派或者其他的Linux系统安装 mosquito  

sudo apt-get install mosquitto-dev  
sudo apt-get install mosquitto-clients  
然后在 /etc/mosquitto/mosquitto.conf  添加支持wensocket的协议和端口  
port 1883  
listener 8083  
protocol websockets  

请在浏览器打开每个功能模块下的 xx.html 正常使用。  
使用前使用notepad++  编辑xx.html中的hostip 地址改成你的树莓派 或者 orangepi  nanopi的IP地址。  
保存后使用浏览器打开后才能正常连接。   

例如，找到 trajectmap 文件夹，使用文本编辑器打开 index_indoor.html   
找到  MQTTconnect() 函数，将serverip 参数改成mqtt 的serverIp地址。  
然后双击使用网页打开，查看是否正常连接（出现绿色背景）  




目前能用的功能有  
# 指南针功能  
![compass](https://pic2.zhimg.com/v2-b5a9d596113bdbfc3550533f12df0295_r.jpg)  
# 路径和航向显示 trajectorymap
![trajectorymap](https://pic3.zhimg.com/v2-84c0dfa714909f703525771918b2e64e_r.jpg)  
#  户外建图道路网络
![compass](https://github.com/horo2016/easyMQOS/blob/main/web/%E5%BE%AE%E4%BF%A1%E6%88%AA%E5%9B%BE_20230608102444.png)  


