
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

width = 600 # Width Of The Game Window
height = 600  # Height Of The Game Window

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



def getcmd(v,a):
    data = {"control":"1","vel":"%s"%(v),"ang":"%s" %(a)
           }
    return  data
    
def PointsFromAngle(angle):
    ### Returns The Unit Direction Vector With Given Angle ###
    return math.cos(angle),math.sin(angle)

def AngleBetweenAndSide(vector1, vector2):
    ### Returns The Angle Between Vectors And The Side Of Resultant Vector ###
    vector1 = np.array(vector1)
    vector2 = np.array(vector2)
    side = 1 if(np.dot(vector1, vector2) < 0) else -1
    return side,np.arccos(np.clip(np.dot(vector1, vector2), -1.0, 1.0))
    
def Move():
    global  inference_flg
    global last_time
    #cmdVel={"control":"1","vel":"0.1","ang":"0.1"};
    #json_data = json.dumps(cmdVel)
    #print(type(json_data))
    #print(cmdVel.vel)
    while True:
       
        if(inference_flg ==1):
            
            #_SensorsDatas = []
            _SensorsDatas = np.append(SensorsData, 30)
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
                if(DetectCrash > 0):
                    SignalData = _SensorsDatas[:-2]
                    if(sum(SignalData[:2]) > sum(SignalData[-2:])):
                        #DetectCrash = 3
                        cmd =getcmd("0.0","-0.2")
                        json_data = json.dumps(cmd)
                        result = client.publish(cmd_vel_topic,json_data,0)
                        # result: [0, 1]
                        status = result[0]
                        if status == 0:
                            print(f"Send  to topic `{cmd_vel_topic}`")
                        else:
                            print(f"Failed to send message to topic {cmd_vel_topic}")
                    else:#右转
                        DetectCrash = 4
                        cmd =getcmd("0.0","-0.2")
                        json_data = json.dumps(cmd)
                        result = client.publish(cmd_vel_topic,json_data,0)
                        # result: [0, 1]
                        status = result[0]
                        if status == 0:
                            print(f"Send  to topic `{cmd_vel_topic}`")
                        else:
                            print(f"Failed to send message to topic {cmd_vel_topic}")
                      
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
                        #print(type(json_data))
                
                print(DetectCrash)
                SignalData = _SensorsDatas[:-2]
                time.sleep(0.1)
            SensorsData.clear()
            inference_flg =0
        else:
            time.sleep(0.1)#10ms
        #t=time.time()
        #print (int(round(t * 1000)))
     
#作为子线程开启
th = threading.Thread(target=Move)
th.setDaemon(True)
th.start()

class Robot:
    def __init__(self,left_value,right_value):
        self.wideth=100
        self.linear_velocity =0
        self.ang_velocity = 0
        self.lValue=left_value
        self.rValue=right_value

    def get_linera_velocity(self):
        self.linear_velocity =(self.rValue + self.lValue) / 2
        return self.linear_velocity

    def get_linera_velocity(self):
        return (self.rValue - self.rValue) / self.wideth
    def run(self):
        print(self.linear_velocity,self.ang_velocity)


class BotEnv:    
    def __init__(self):
        ### Initializing Environment Variables ###
        global BotStartLocation
        self.robot=Robot(0,0)
        self.crashed = False
        self.DetectCrash = 0
        

        
    def _step(self, action, CrashStep=0):
        #左转
        if action == 3:
            self.robot.ang_velocity =0.1 # 左转
            self.robot.linear_velocity = 0
        ## 右转
        elif action == 4:
            self.robot.ang_velocity = -0.1  #
            self.robot.linear_velocity = 0
        ## 直行
        elif action == 5:
            self.robot.ang_velocity = 0  #
            self.robot.linear_velocity = 0.2
        self.robot.run()
        ## Get The Current Location And The Sensors Data At Given Point.
        #x, y = self.Bot.position        ## Get The Bot Position
        SensorsData = self.AllSensorSensorsData(0, 0, 0)  ## Get All The Sensor Data
        SensorsData = np.append(SensorsData, 30)
        SensorsData = np.append(SensorsData, [0])
        print(SensorsData[:-2])  ## Print The Sensor Data
        DataTensor = torch.Tensor(SensorsData[:-1]).view(1,-1)
        if (model != None):
            ## Get Decision From Neural Network If There Is A Collison
            self.DetectCrash = model(Variable(DataTensor))
            print("inference:")
            print(self.DetectCrash.data)
            self.DetectCrash = abs(np.round(self.DetectCrash.data[0][0]))
            print(self.DetectCrash)
            if(self.DetectCrash > 0):
                SignalData = SensorsData[:-2]
                if(sum(SignalData[:2]) > sum(SignalData[-2:])):
                 self.DetectCrash = 3
                else:
                 self.DetectCrash = 4
            
       
        SignalData = SensorsData[:-2]
        return

    def AllSensorSensorsData(self, x, y, angle):
        SensorsData = []
        NumberOfSensors = 5

        ## Generate Sensors
        ## 从传感器中读取数据
        for i in range(NumberOfSensors):
            SensorsData.append(int(self.SensorReading(i, x, y, angle)))
        
        return SensorsData

    def SensorReading(self, sensor, x, y, angle):
        ###随机产生数据 ###
        n = random.randint(1,10)
        if (n%2)==0:
            distance = random.randint(10,101)
        else:
            distance =100
        return distance

        
    def Rotate(self,origin, point, angle):
        ###控制机器人旋转 ###
        x1, y1 = origin
        x2, y2 = point
        final_x = x1 + math.cos(angle) * (x2 - x1) - math.sin(angle) * (y2 - y1)
        final_y = y1 + math.sin(angle) * (x2 - x1) + math.cos(angle) * (y2 - y1)
        final_y = abs(width - final_y)
        return final_x,final_y
        
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
    '''for i in range(2000):
        
        if (env.DetectCrash > 0):
            DrivingSide = env.DetectCrash
            
            for i in range(3):env._step(DrivingSide)
        else:
            x = 5
            env._step(x)'''

