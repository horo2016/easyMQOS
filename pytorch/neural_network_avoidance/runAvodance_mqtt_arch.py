
import pdb
import random
import math
import numpy as np
from makeLearn_aarch import *
import sys
import json
from paho.mqtt import client as mqtt_client
import time
import struct 
import threading


SummarySensorData = []
StepSizeValue = 1/10 # Step Size For Simulation
ClockTickValue = 25  # Clock Tick
BotSpeed = 20 # Speed Of The Bot
model = Net(InputSize, NumClasses)
model.load_state_dict(torch.load('./NNBot.pth'))# my ANN


#MQTT相关
broker = '127.0.0.1'
#broker = 'www.woyilian.com'
port = 1883
lidar_topic = "/sensors/lidar_node_pub"
odom_topic = "/sensors/odom"
cmd_vel_topic = "/cmd/vel"

client_id = 'python-mqtt-{}'.format(random.randint(0, 1000))
#定义mqtt的client
client = mqtt_client.Client(client_id)

SensorsData = []
NumberOfSensors = 5
inference_flg =0

BotStartLocation =  2
DetectCrash =0


def getcmd(v,a):
    data = {"control":"1","vel":"%s"%(v),"ang":"%s" %(a)
           }
    return  data
    

def Inference():
    global  inference_flg
    global last_time

    while True:
        if(inference_flg ==1):
            _SensorsDatas = np.append(SensorsData, 30)#小车角度
            _SensorsDatas = np.append(_SensorsDatas, [0])
            
            print(_SensorsDatas[:-2])  ## Print The Sensor Data
            DataTensor = torch.Tensor(_SensorsDatas[:-1]).view(1,-1)
            if (model != None):
                ## Get Decision From Neural Network If There Is A Collison
                DetectCrash = model(Variable(DataTensor))
                print("inference:")
                #print(DetectCrash.data)
                DetectCrash = abs(np.round(DetectCrash.data[0][0]))
                print(DetectCrash)
                if(DetectCrash > 0):#为1时
                    SignalData = _SensorsDatas[:-2]
                    if(sum(SignalData[:2]) > sum(SignalData[-2:])):#左边的空间比右边的空间大
                        DetectCrash = 3
                    else:#右转
                        DetectCrash = 4
                
                print(DetectCrash)
                SignalData = _SensorsDatas[:-2]
                time.sleep(0.01)
            SensorsData.clear()
            inference_flg =0
        else:
            time.sleep(0.1)#100ms
        #t=time.time()
        #print (int(round(t * 1000)))
     
#作为子线程开启
th = threading.Thread(target=Inference)
th.setDaemon(True)
th.start()

def Move():
    
    cmd =getcmd("0.0","0.0") 
    while True:
       
        if (DetectCrash > 0):
            
            for i in range(3):
                if (DetectCrash==3):
                    cmd =getcmd("0.0","-0.2")
                else:
                    cmd =getcmd("0.0","0.2")
                json_data = json.dumps(cmd)
                result = client.publish(cmd_vel_topic,json_data,0)
                # result: [0, 1]
                status = result[0]
                if status == 0:
                    print(f"Send  to topic `{cmd_vel_topic}`")
                else:
                    print(f"Failed to send message to topic {cmd_vel_topic}")
                time.sleep(0.1)
        else:#前进
            cmd =getcmd("0.1","0")
            json_data = json.dumps(cmd)
            result = client.publish(cmd_vel_topic,json_data,0)
            # result: [0, 1]
            status = result[0]
            if status == 0:
                print(f"Send  to topic `{cmd_vel_topic}`")
            else:
                print(f"Failed to send message to topic {cmd_vel_topic}")
        time.sleep(0.01)
        
#作为子线程开启
th1 = threading.Thread(target=Move)
th1.setDaemon(True)
th1.start()


        
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("test")

def on_message(client, userdata, message):
    global _switch
    global  inference_flg
    #print(message.topic+" "+message.payload.decode("utf-8"))
    #_switch = ord(message.payload.decode("utf-8"))-48
    #print("%d"%_switch)
    #print("Receiving message")
    print(message.topic)
    if (message.topic == lidar_topic):
        #bydat = message.payload.decode("utf-8")
        A =struct.unpack(str(len(message.payload))+'B', message.payload)
        #save_payload(message.payload, pic_filename)
        #print(A[85])
        #print(len(A))
       
        if(inference_flg == 0):
            
            #SensorsData.clear()
            v =(A[85]*256+A[84])/10
            if v>100:
                v=100
            SensorsData.append(v)#40
            v =(A[45]*256+A[44])/10
            if v>100:
                v=100
            SensorsData.append(v)#20
            v =(A[9]*256+A[8])/10
            if v>100:
                v=100
            SensorsData.append(v)#0
            v = (A[685]*256+A[684])/10
            if v>100:
                v=100
            SensorsData.append(v)#360
            v = (A[645]*256+A[644])/10
            if v>100:
                v=100
            SensorsData.append(v)#340
            inference_flg =1
        
    if (message.topic == odom_topic):
        #save_payload(message.payload, vid_filename)
        print(message.payload)
         
if __name__ == "__main__":
    #env = BotEnv()
    #random.seed(10)
    
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port, 60)
    
    # 订阅主题
    client.subscribe(lidar_topic)
    #publish(client)
    client.loop_forever()

