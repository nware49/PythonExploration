import threading
import queue
import logging
import time
import random
import serial

PortChannel1 = "/dev/ttySC0"
PortChannel2 = "/dev/ttySC1"

logging.basicConfig(level=logging.DEBUG,
                    format='(%(threadName)-9s) %(message)s',)
try:
    Serial1 = serial.Serial(PortChannel1)
    Serial2 = serial.Serial(PortChannel2)
except:
    print("Could not open port. Port is either in use or does not exist.")
    exit()

BUF_SIZE = 10
q1 = queue.Queue(BUF_SIZE)
q2 = queue.Queue(BUF_SIZE)

class ProducerThreadChannel1(threading.Thread):
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(ProducerThreadChannel1,self).__init__()
        self.target = target
        self.name = name

    def run(self):
        while True:
            if not q1.full():
                ser_bytes = Serial1.readline()
                DecodedBytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
                DecodedBytes = DecodedBytes
                #print(DecodedBytes)
                IncrementNum,TimeVal,RandFloat,SineVal,CosineVal = DecodedBytes.split(",")
                q1.put(IncrementNum)
                data = IncrementNum

        return

class ConsumerThreadChannel1(threading.Thread):
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(ConsumerThreadChannel1,self).__init__()
        self.target = target
        self.name = name
        return

    def run(self):
        while True:
            if not q1.empty():
                data = q1.get()
                print(data)
        return

class ProducerThreadChannel2(threading.Thread):
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(ProducerThreadChannel2,self).__init__()
        self.target = target
        self.name = name

    def run(self):
        while True:
            if not q2.full():
                ser_bytes = Serial2.readline()
                DecodedBytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
                DecodedBytes = DecodedBytes
                #print(DecodedBytes)
                IncrementNum,TimeVal,RandFloat,SineVal,CosineVal = DecodedBytes.split(",")
                q2.put(IncrementNum)
                data = IncrementNum
        return

class ConsumerThreadChannel2(threading.Thread):
    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None, verbose=None):
        super(ConsumerThreadChannel2,self).__init__()
        self.target = target
        self.name = name
        return

    def run(self):
        while True:
            if not q2.empty():
                data = q2.get()
                print(data)
        return

try:
    if __name__ == '__main__':
        
        pc1 = ProducerThreadChannel1(name='producerChannel1')
        cc1 = ConsumerThreadChannel1(name='consumerChannel1')
        pc2 = ProducerThreadChannel2(name='producerChannel2')
        cc2 = ConsumerThreadChannel2(name='consumerChannel2')

        pc1.start()
        pc2.start()
        time.sleep(2)
        cc1.start()
        cc2.start()
        
except KeyboardInterrupt:
    print("Keyboard Interrupt")
    exit()
except:
    print("An error occurred, data could not be recieved.")
    exit()
