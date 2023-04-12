import cv2
import numpy as np
import argparse
import os
import sys
import datetime
from paho.mqtt import client as mqtt_client
import random
import time
import threading
#MQTT相关
broker = '127.0.0.1'
#broker = 'www.woyilian.com'
port = 1883
topic = "/camera/collect"
client_id = 'python-mqtt-{}'.format(random.randint(0, 1000))



 
orgFrame = None
ret = False
Running = True
detect_ok = False
resultImg =None
def Camera_isOpened():
    global  cap
        #开始捕获摄像头视频流
    cap = cv2.VideoCapture('/dev/video0')
try:
    Camera_isOpened()
except:
    print('Unable to detect camera! \n')
    
def get_image():
    global orgFrame
    global ret
    global Running
    global  cap
 
    while True:
        if Running:
            if cap.isOpened():
                ret, orgFrame = cap.read()
                time.sleep(0.001)
            else:
                time.sleep(0.01)
        else:
            time.sleep(0.01)

th1 = threading.Thread(target = get_image)
th1.setDaemon(True)
th1.start()
# sigmoid函数
def sigmoid(x):
    return 1. / (1 + np.exp(-x))

# tanh函数
def tanh(x):
    return 2. / (1 + np.exp(-2 * x)) - 1
# nms算法
def nms(dets, thresh=0.45):
    # dets:N*M,N是bbox的个数，M的前4位是对应的（x1,y1,x2,y2），第5位是对应的分数
    # #thresh:0.3,0.5....
    x1 = dets[:, 0]
    y1 = dets[:, 1]
    x2 = dets[:, 2]
    y2 = dets[:, 3]
    scores = dets[:, 4]
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)  # 求每个bbox的面积
    order = scores.argsort()[::-1]  # 对分数进行倒排序
    keep = []  # 用来保存最后留下来的bboxx下标

    while order.size > 0:
        i = order[0]  # 无条件保留每次迭代中置信度最高的bbox
        keep.append(i)

        # 计算置信度最高的bbox和其他剩下bbox之间的交叉区域
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        # 计算置信度高的bbox和其他剩下bbox之间交叉区域的面积
        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h

        # 求交叉区域的面积占两者（置信度高的bbox和其他bbox）面积和的必烈
        ovr = inter / (areas[i] + areas[order[1:]] - inter)

        # 保留ovr小于thresh的bbox，进入下一次迭代。
        inds = np.where(ovr <= thresh)[0]

        # 因为ovr中的索引不包括order[0]所以要向后移动一位
        order = order[inds + 1]
    
    output = []
    for i in keep:
        output.append(dets[i].tolist())

    return output
def detect():
    global Running
    global orgFrame, ret   
    global detect_ok
    global resultImg
    net = cv2.dnn.readNetFromONNX('FastestDet.onnx')
    circles_list = []
    color_list = []
    count = 0
    while True:     
      if orgFrame is not None and ret:
        if Running:
            t1 = cv2.getTickCount()
            orgframe = cv2.resize(orgFrame,(512, 256))
            t1 = cv2.getTickCount()
            blob = cv2.dnn.blobFromImage(orgframe, 1 / 255.0, (352, 352))
            net.setInput(blob)
            layer = net.getUnconnectedOutLayersNames()#获取最后一层 
            #layeralls= net.getLayerNames()#获取所有的输出层名称
            #print(layer)
            #前向传播获得信息
            pred = net.forward(layer) 
            #model = FastestDet(confThreshold=args.confThreshold, nmsThreshold=args.nmsThreshold)
            #print( type(pred))#查看类型 class tuple
            #print( len(pred))#查看元组长度 1
            #print( type(pred[0]))#查看元组0的类型
            array_output= pred[0]
            #print( array_output.ndim) #查看维度：4维
            #print(array_output.shape)#输出行数和列数 1x9x22x22 与netron 查看结果一样
            #print(array_output.size)#输出总共有多少元素  上边相乘的结果 
            #print( array_output[0].ndim)
            #print(array_output[0].shape)#输出行数和列数 9x22x22 与netron 查看结果一样
            #print(array_output[0].size)#输出总共有多少元素  上边相乘的结果 
            #print(array_output[0]) 
            
            t3 = cv2.getTickCount()
            sec = (t3 - t1)
            label = 'Inference time: %.2f ms' % (sec * 1000.0 /  cv2.getTickFrequency())
            print(label)
            classIds = []
            confidences = []
            boxes =  []
            box  = []
            frameHeight = orgframe.shape[0]
            frameWidth = orgframe.shape[1]
            feature_map = array_output[0]
            # 输出特征图转置: CHW-> HWC
            feature_map = feature_map.transpose(1, 2, 0)
            # 输出特征图的宽高
            feature_map_height = feature_map.shape[0]
            feature_map_width = feature_map.shape[1]
            #print(feature_map_height)
            #print(feature_map_width) 
            # 特征图后处理
            for h in range(feature_map_height):
                for w in range(feature_map_width):
                    data = feature_map[h][w]

                    # 解析检测框置信度
                    obj_score, cls_score = data[0], data[5:].max()
                    score = (obj_score ** 0.6) * (cls_score ** 0.4)
                    #print(score)
                    # 阈值筛选
                    if score > 0.65:
                        # 检测框类别
                        cls_index = np.argmax(data[5:])
                        # 检测框中心点偏移
                        x_offset, y_offset = tanh(data[1]), tanh(data[2])
                        # 检测框归一化后的宽高
                        box_width, box_height = sigmoid(data[3]), sigmoid(data[4])
                        # 检测框归一化后中心点
                        box_cx = (w + x_offset) / feature_map_width
                        box_cy = (h + y_offset) / feature_map_height
                        
                        # cx,cy,w,h => x1, y1, x2, y2
                        x1, y1 = box_cx - 0.5 * box_width, box_cy - 0.5 * box_height
                        x2, y2 = box_cx + 0.5 * box_width, box_cy + 0.5 * box_height
                        x1, y1, x2, y2 = int(x1 * frameWidth), int(y1 * frameHeight), int(x2 * frameWidth), int(y2 * frameHeight)
                        confidences.append(float(score))
                        box.append([x1, y1, x2, y2])
                        
                        #cv2.rectangle(srcimg, (x1, y1), (  x2, y2), (0, 255, 0), thickness=2)
                        boxes.append([x1, y1, x2, y2, score, cls_index])
            #print(boxes )
            #print(len(boxes))
            if len(boxes) != 0: 
                bboxes =nms(np.array(boxes))
                for b in bboxes:
                    #print(b)
                    obj_score, cls_index = b[4], int(b[5])
                    x1, y1, x2, y2 = int(b[0]), int(b[1]), int(b[2]), int(b[3])

                    #绘制检测框
                    cv2.rectangle(orgframe, (x1,y1), (x2, y2), (0, 0, 0), 1)
                    cv2.putText(orgframe, '%.2f' % obj_score, (x1, y1 - 5), 0, 0.7, (0, 255, 0), 1)
                    cv2.putText(orgframe, classes[cls_index], (x1, y1 - 25), 0, 0.7, (0, 255, 255), 2)
                detect_ok = True
                resultImg=orgframe.copy()
            time.sleep(0.001)
            #cv2.imwrite("res_cv.jpg", orgframe)
        else:
            time.sleep(0.01)
#启动检测的线程
th2 = threading.Thread(target=detect)
th2.setDaemon(True)
th2.start()
#定义mqtt的client
client = mqtt_client.Client(client_id)
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
    global detect_ok

    global resultImg
    msg_count = 0
    while True:
        if detect_ok is True:
            detect_ok = False
            #重新裁剪大小
            #img = cv2.resize(img,(512,256))
            #if _switch == 0:#如果为0 直接重头开始
                #continue
            #cv2.imshow("IN", img)
            #是否写入保存的文件
            #if args.savevideo:
                #out.write(orgFrame)
            t=time.time()
            #print (int(round(t * 1000)))
            img_encode = cv2.imencode('.jpg', resultImg)[1] #.jpg 编码格式
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
            time.sleep(0.001)
            if (t*1000 - last_time*1000 >= 500):
                result = client.publish(topic,byteArr,0)
                last_time = t
                # result: [0, 1]
                status = result[0]
                if status == 0:
                    print(f"Send  to topic `{topic}`")
                else:
                    print(f"Failed to send message to topic {topic}")
        else:
            time.sleep(0.01)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--imgpath', type=str, default='w.jpg', help="image path")
    
    args = parser.parse_args()
    classes = list(map(lambda x: x.strip(), open('coco.names',
                                                          'r').readlines()))
    #是否保存为AVI视频
    #if args.savevideo:
        #out = cv2.VideoWriter("road" + ".avi", cv2.VideoWriter_fourcc('M','J','P','G'),10,(512,256))

    print("camera connected!")        
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port, 60)
    publish(client)
    # 订阅主题
    client.subscribe(topic)
    client.loop_forever()
    
    
    
    '''
    indices = cv2.dnn.NMSBoxes(box, confidences, 0.2, 0.3)
    for i in indices:
            #i = i[0]
            bx = box[i]
            left = bx[0]
            top = bx[1]
            width = bx[2]
            height = bx[3]
             
            # Draw a bounding box.
            cv2.rectangle(srcimg, (left, top), (  width, height), (0, 0, 255), thickness=2)
    '''
    # Perform non maximum suppression to eliminate redundant overlapping boxes with
    # lower confidences.
    #winName = 'Deep learning object detection in OpenCV'
    #cv2.namedWindow(winName, 0)
    
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

