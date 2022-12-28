#  说明
该程序配合mqtt使用 ；
在树莓派或者其他派终端安装mqtt 工具包。  

首先在树莓派或者其他的Linux系统安装 mosquito

sudo apt-get install mosquitto-dev
sudo apt-get install mosquitto-clients
然后在 /etc/mosquitto/mosquitto.conf  添加支持wensocket的协议和端口
port 1883
listener 8083
protocol websockets

请在浏览器打开每个功能模块下的 xx.html 正常使用。  
使用前使用notepad++  编辑xx.html中的hostip 地址改成你的树莓派 或者 orangepi  nanopi的IP地址。保存后使用   
浏览器打开后才能正常连接。

目前能用的功能有  
# 指南针功能  
![compass](https://pic2.zhimg.com/v2-b5a9d596113bdbfc3550533f12df0295_r.jpg)  
# 路径和航向显示 trajectorymap
![trajectorymap](https://pic3.zhimg.com/v2-84c0dfa714909f703525771918b2e64e_r.jpg)  
#  twist 键盘控制功能


