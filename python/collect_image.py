import os
import sys
import argparse
import cv2
import datetime
from paho.mqtt import client as mqtt_client
import random
import time
#MQTT相关
#broker = '127.0.0.1'
broker = 'www.woyilian.com'
port = 1883
topic = "/camera/collect"
client_id = 'python-mqtt-{}'.format(random.randint(0, 1000))
#接收开关
_switch =0 
#参数相关
parser = argparse.ArgumentParser(description='collect data')
parser.add_argument('--savevideo', type=str, default='', help='save incoming video')
args = parser.parse_args()
print(args)
#定义mqtt的client
client = mqtt_client.Client(client_id)

framecount = 0
last_time =0
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("test")

def on_message(client, userdata, msg):
    global _switch
    #print(msg.topic+" "+msg.payload.decode("utf-8"))
    _switch = ord(msg.payload.decode("utf-8"))-48
    print("%d"%_switch)
def publish(client):
    global last_time
    msg_count = 0
    while True:
        ret, img = cap.read()
        if not ret:
            break
        #重新裁剪大小
        img = cv2.resize(img,(512,256))
        #if _switch == 0:#如果为0 直接重头开始
            #continue
        #cv2.imshow("IN", img)
        #是否写入保存的文件
        if args.savevideo:
            out.write(img)
        t=time.time()
        #print (int(round(t * 1000)))
        img_encode = cv2.imencode('.jpg', img)[1] #.jpg 编码格式
        #t=time.time()
        #print (int(round(t * 1000)))
        
        #curr_time = datetime.datetime.now()
        #time_str = datetime.datetime.strftime(curr_time,'%Y%m%d%H%M%S')
        
        #framecount += 1
        byteArr = bytearray(img_encode)#1xN维的数据
        #print(byteArr)
        #print(len(byteArr))
        #每1秒钟采集一次
        #cv2.imwrite("img/%s.jpg"%time_str, img) 
        if (t*1000 - last_time*1000 >= 500):
            result = client.publish(topic,byteArr,0)
            last_time = t
            # result: [0, 1]
            status = result[0]
            if status == 0:
                print(f"Send  to topic `{topic}`")
            else:
                print(f"Failed to send message to topic {topic}")

#是否保存为AVI视频
if args.savevideo:
    out = cv2.VideoWriter("road" + ".avi", cv2.VideoWriter_fourcc('M','J','P','G'),10,(512,256))
#开始捕获摄像头视频流
cap = cv2.VideoCapture('/dev/video0')
print("camera connected!")        
client.on_connect = on_connect
client.on_message = on_message
client.connect(broker, port, 60)
publish(client)
# 订阅主题
client.subscribe(topic)
client.loop_forever()




    