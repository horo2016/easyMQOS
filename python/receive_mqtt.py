import time
import paho.mqtt.client as paho
broker="192.168.3.8"
pic_topic="myhome/dafang/motion/snapshot/image"
pic_filename="D:/Dump/motion-snap.jpg"
vid_topic="/camera/collect"
vid_filename="motion-vid.mp4"
port = 1883
cnt =0
def on_message(client, userdata, message):
    #print("Receiving message")
    print(message.topic)
    if (message.topic == pic_topic):
        save_payload(message.payload, pic_filename)
    if (message.topic == vid_topic):
        save_payload(message.payload, vid_filename)
def save_payload(payload, filename):
    global cnt
    cnt =cnt+1
    print("img%d\n"%cnt)
    #print("Saving file: "+filename)
    f=open(filename, "ab") # 'w' for 'write', 'b' for 'write as binary, not text'
    f.write(payload)
    f.close()
print("Starting")
c=paho.Client("clientddafddfsd188")
c.connect(broker, port, 60)
c.subscribe(pic_topic)
c.subscribe(vid_topic)
c.on_message = on_message
c.loop_forever()