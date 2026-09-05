# Usart Library

import serial
import struct
import binascii
from time import sleep
import threading #开启线程 接收串口数据 20230505  Open thread, receive serial port data 20230505
import datetime
import time

# Init serial port
Usart = serial.Serial()
Usart = serial.Serial(
    #port='/dev/ttyUSB0',  # 串口 linux 开启串口 Serial port,linux Enables the serial port
    port='com5',  # 串口 依据实际情况开启串口 Serial port,enable the serial port as required
    baudrate=115200,  # 波特率 Baud rate
    timeout=0.001)
time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
print(time)

#关闭串口  Closes the serial port
def port_close():
    Usart.close()
    if Usart.isOpen():
        time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        print("Serial port closed failure！"+ time)
    else:
        time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        print("Serial port closed successful！"+ time)

#将浮点数转化为4个unsigned char 类型的数据 Converts the floating-point number to four unsigned char types
def DataAnla(f):
    kk = [0xA4, 0x03, 0x08, 0x23]  # 需要发送的串口包 Serial port packet to be sent
    print(kk)
   # kk = struct.pack("%dB" % (len(kk)), *kk)  # 解析成16进制 Parse into hexadecimal
   # print(kk)

    print(f)
    if f < 0:    #f中的内容为负数 The contents of f are negative
        f = abs(f)  # 将F中的内容转换为正数 Convert the contents of F to positive numbers
        a = int(f)
        print(a)
        kk[0] = int(a / 256) + 0x80    # 将F中的内容转换为负数 Convert the contents of F to a negative number
        print(int(a / 256) + 0x80)
        print(kk[0])
        kk[1] = a % 256
        print(kk[1])
        print("f;")
        print(f);
        print("a=")
        print(a);
        a = int((f-a+0.00001) * 10000) #减小误差 error reduction
        print("f - a")
        print(a)
        kk[2] = int(a / 256)
        print(kk[2])
        kk[3] = a % 256
        print(kk[3])
        kk = struct.pack("%dB" % (len(kk)), *kk)  # 解析成16进制 Parse into hexadecimal
        #Usart.write(kk)
        #print(kk)
    else:
        a = int(f)
        print(a)
        kk[0] = int(a / 256)
        print(int(a/256))
        print(kk[0])
        kk[1] = a % 256
        print(kk[1])
        print((f-a))
        a = int((f-a+0.00001) * 10000) #减小误差 error reduction
        print(a)
        kk[2] = int (a / 256)
        print(kk[2])
        kk[3] = a % 256
        print(kk[3])

    kk = struct.pack("%dB" % (len(kk)), *kk)  # 解析成16进制 Parse into hexadecimal
    #Usart.write(kk)
    print(kk)
    return kk #将浮点数转化为4个unsigned char 类型的数据 Converts the floating-point number to four unsigned char types

def CalData(kk0,kk1,kk2,kk3):
                            d =0.0
                            #Flag = int(kk0)&int(0x80)
                            #if Flag:
                            #        kk0-=0x80
                             #       d=double(kk0*256+kk1+(kk2*256+kk3)*0.0001)
                                    # d=-1*d
                            #else:
                             #   a = 0
                             #   d=double(kk0*256+kk1+(kk2*256+kk3)*0.0001)
                            d = (kk0 * 256 + kk1 + (kk2 * 256 + kk3)*0.00001)
                            shuju = 123456789
                            print(shuju)
                            print(d)
                            return d


def sendVf(f):
    sendArr = [0xaa, 0x01, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    kk = DataAnla(f) #将浮点数 编码 Encode floating-point numbers
    sendArr[6] = kk[0]
    sendArr[7] = kk[1]
    sendArr[8] = kk[2]
    sendArr[9] = kk[3]
    print(sendArr)
    Xor = 0x00
    # 抑或校验位 BCC(Block Check Character）
    for i in range(0,10):
        #print(i)
        Xor = sendArr[i] ^ Xor
        print(Xor)
    sendArr[10]= Xor
    print("2SendArr")
    print(sendArr)
    sendArr = struct.pack("%dB" % (len(sendArr)), *sendArr)  # 解析成16进制 Parse into hexadecimal
    Usart.write(sendArr)
    print(sendArr)


def sendMovef(f):
    sendArr = [0xaa, 0x01, 0x0b, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    kk = DataAnla(f) #将浮点数 编码 Encode floating-point numbers
    sendArr[6] = kk[0]
    sendArr[7] = kk[1]
    sendArr[8] = kk[2]
    sendArr[9] = kk[3]
    print(sendArr)
    # 抑或校验位 BCC(Block Check Character）
    Xor = 0x00
    for i in range(0,10):
        #print(i)
        Xor = sendArr[i] ^ Xor
        print(Xor)
    sendArr[10]= Xor
    print("2SendArr")
    print(sendArr)
    sendArr = struct.pack("%dB" % (len(sendArr)), *sendArr)  # 解析成16进制 Parse into hexadecimal
    Usart.write(sendArr)
    print(sendArr)
# 串口接收数据 Parse into hexadecimal
def recv(serial):
        while True:
            data = serial.read_all()
            if data == '':
                continue
            else:
                break
            sleep(0.02)
        return data
def sentOpenOrClose(add,B3,FlagOpenOrClosed,Ch):
    sendArr = [0xaa, 0x01, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00]
    B4=0x00
    #sendArr = [0xaa, add, 0x0b, B3, B4, Ch, FlagOpenOrClosed, 0x00]
    sendArr[1] = int(add)
    sendArr[3] = int(B3)
    sendArr[4] = int(B4)
    sendArr[5] = int(Ch)
    sendArr[6] = int(FlagOpenOrClosed)

    print(sendArr)
    # 抑或校验位 BCC(Block Check Character）
    Xor = 0x00
    for i in range(0, 7):
        # print(i)
        Xor = sendArr[i] ^ Xor
        print(Xor)
    sendArr[7] = Xor
    print("2SendArr")
    print(sendArr)
    sendArr = struct.pack("%dB" % (len(sendArr)), *sendArr)  # 解析成16进制 Parse into hexadecimal
    Usart.write(sendArr)
    print(sendArr)

#读取开闭环
def ReadOpenOrClsoe(add,B3,Ch):
    sendArr = [0xaa, 0x01, 0x07, 0x13, 0x00, 0x00, 0xbf]
    B4 = 0x00
    # sendArr = [0xaa, add, 0x0b, B3, B4, Ch, FlagOpenOrClosed, 0x00]
    sendArr[1] = int(add)
    sendArr[3] = int(B3)
    sendArr[4] = int(B4)
    sendArr[5] = int(Ch)
    sendArr = struct.pack("%dB" % (len(sendArr)), *sendArr)  # 解析成16进制 Parse into hexadecimal
    Usart.write(sendArr)
    sleep(0.02)
    readArr = recvClosedOrOpen(Usart,8)
    print(readArr[6])
    return readArr[6]


def recvClosedOrOpen(Usart,LongData):
    data = Usart.read(LongData)  # 是读20个字符
    print(data)
    return data

def sentDigitalorAnalog(add,B3,FlagOpenOrClosed,Ch):
    sendArr = [0xaa, 0x01, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00]
    B4=0x00
    #sendArr = [0xaa, add, 0x0b, B3, B4, Ch, FlagOpenOrClosed, 0x00]
    sendArr[1] = int(add)
    sendArr[3] = int(B3)
    sendArr[4] = int(B4)
    sendArr[5] = int(Ch)
    sendArr[6] = int(FlagOpenOrClosed)

    print(sendArr)
    # 抑或校验位 BCC(Block Check Character）
    Xor = 0x00
    for i in range(0, 7):
        # print(i)
        Xor = sendArr[i] ^ Xor
        print(Xor)
    sendArr[7] = Xor
    print("2SendArr")
    print(sendArr)
    sendArr = struct.pack("%dB" % (len(sendArr)), *sendArr)  # 解析成16进制 Parse into hexadecimal
    Usart.write(sendArr)
    print(sendArr)

#读取开闭环
def ReadDigitalorAnalog(add,B3,Ch):
    sendArr = [0xaa, 0x01, 0x07, 0x13, 0x00, 0x00, 0xbf]
    B4 = 0x00
    # sendArr = [0xaa, add, 0x0b, B3, B4, Ch, FlagOpenOrClosed, 0x00]
    sendArr[1] = int(add)
    sendArr[3] = int(B3)
    sendArr[4] = int(B4)
    sendArr[5] = int(Ch)

    Xor = 0x00
    for i in range(0, 6):
        # print(i)
        Xor = sendArr[i] ^ Xor
        print(Xor)
    sendArr[6] = Xor
    sendArr = struct.pack("%dB" % (len(sendArr)), *sendArr)  # 解析成16进制 Parse into hexadecimal
    Usart.write(sendArr)
    sleep(0.05)
    readArr = recvDigitalorAnalog(Usart,8)
    print(readArr[6])
    return readArr[6]

def recvDigitalorAnalog(Usart,LongData):
    data = Usart.read(LongData)  # 是读20个字符
    print(data)
    return data
def recvPro(Usart):
    data = Usart.read(11)  # 是读20个字符
    print(123456)
    print(data)
    return data
def readVoltOrMove(add,B3,Ch):
    #sendArr = [0xaa, 0x01, 0x07, 0x05, 0x00, 0x00, 0x09]
    sendArr = [0xaa, 0x01, 0x07, B3, 0x00, Ch, 0x09]
    Xor = 0x00
    for i in range(0, 6):
        # print(i)
        Xor = sendArr[i] ^ Xor
        print(Xor)
    sendArr[6] = Xor
    print("2SendArr")
    print(sendArr)
    sendArr = struct.pack("%dB" % (len(sendArr)), *sendArr)  # 解析成16进制 Parse into hexadecimal
    Usart.write(sendArr)
    sleep(0.09)#50ms sleep
    readDataArr = [0xaa,0x01,0x0b,0x06,0x00,0x00,0x00,0x05,0x08,0x04,0x6f]
    readDataArr = recvPro(Usart)
    #readDataArr = recv(Usart)

    # print readDataArr
    print(readDataArr)
    # tmpArr = hexShow(readDataArr)

    # 异或数据位
    #tmp_channle = readDataArr[5]  # channel 通道号
    read_data = CalData(readDataArr[6],readDataArr[7],readDataArr[8],readDataArr[9])
    #readDataArrT = [0xaa, 0x01, 0x0b, 0x06, 0x00, 0x00, 0x00, 0x05, 0x08, 0x04, 0x6f]
    #a = readDataArrT[5]
   # print(readDataArrT)
   # read_data = CalData(readDataArrT[5], readDataArrT[6], readDataArrT[7], readDataArrT[8])
    read_data = 10.001
    print(read_data)
    return read_data

# 发送数据 send data
def uart_send_data(uart, txbuf):
    len = uart.write(txbuf.encode('utf-8'))  # 写数据 write data
    return len

# 接收数据 receive data

#str = uart.read(uart.in_waiting).decode("utf-8")   # 以字符串接收 Receive as string
#str = uart.read().hex()                            # 以16进制(hex)接收 Receive in hex

def uart_receive_data(uart):
    if uart.in_waiting:
        rxdata = uart.read(uart.in_waiting).decode("utf-8")   # 以字符串接收 Receive as string
        # rxdata = uart.read().hex()  # 以16进制(hex)接收 Receive in hex
        print(rxdata)  # 打印数据 print data

# 创建一个线程用来等待串口接收数据 Create a thread that waits for the serial port to receive data
class myThread (threading.Thread):   # 继承父类threading.Thread  Inherited superclass threading.Thread
    def __init__(self, uart):
        threading.Thread.__init__(self)
        self.uart = uart
    def run(self):                   # 把要执行的代码写到run函数里面 线程在创建后会直接运行run函数 Write the code to be executed into the run function, which the thread runs directly after creation
        while True:
            # print("thread_uart_receive")
            uart_receive_data(self.uart)  # 接收数据 receive data
            # time.sleep(0.01)

def sendTest(uart):
    data1 = "hello world"        # 字符串 character string
    data2 = b"hello world"       # bytes
    data3 = "你好"               # 中文字符串 Chinese string
    data4 = 0x0A                 # 整形(以16进制表示) Shaping (expressed in hexadecimal)
    data5 = [0x10, 0x11, 0x12]   # 列表/数组(以16进制表示) List/array (in hexadecimal)

    len = uart.write(data1.encode('utf-8'))         # 发送字符串"hello world" Send the string "hello world"
    len = uart.write(data2)                         # 发送字符串"hello world" Send the string "hello world"
    len = uart.write(data3.encode('utf-8'))         # 以utf-8编码方式发送字符串"你好"（6字节）Send the string "Hello" (6 bytes) in utf-8 encoding
    len = uart.write(data3.encode('gbk'))           # 以gbk编码方式发送字符串"你好"（4字节） Send the string "Hello" in gbk encoding (4 bytes)
   # len = uart.write(chr(data4.encode("utf-8"))     # 发送16进制数据0x0A（1字节） Send hexadecimal data 0x0A (1 byte)
    #for x in data5:                                 # 遍历列表/数组的所有元素并依次发送 Iterate over all the elements of the list/array and send them in turn
    #    len = uart.write(chr(x).encode("utf-8"))



# 判断串口是否打开成功 Check whether the serial port is successfully enabled
if Usart.isOpen():
    time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print("open success,串口打开成功 " + time)
else:
    time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print("open failed,串口打开失败 " + time)

# 使用优雅的方式发送串口数据 Send serial port data in an elegant manner
# 这里的数据可以根据你的需求进行修改 The data here can be modified to suit your needs
send_data = [0xA4, 0x03, 0x08, 0x23, 0xD2]  # 需要发送的串口包 Serial port packet to be sent

send_data = struct.pack("%dB" % (len(send_data)), *send_data)  # 解析成16进制 Parse into hexadecimal
Usart.write(send_data)
print(send_data)
sleep(0.02)

#要发送的数据依照编码规则转化，10.001数字在开环情况下单位为电压，闭环情况下单位为微米或者或毫弧度
#The data to be sent is converted according to the code rules, with the 10.001 digit expressed in units of voltage in the open loop and microns or milliradians in the closed loop.
#sendArrF = DataAnla(10.001)
#print(sendArrF)
#Usart.write(sendArrF)

#sendArrF = DataAnla(-10.001)
#print(sendArrF)
#Usart.write(sendArrF)

#sendVf(10.001) #发送10.001V Send 10.001V
#sendMovef(10.001) #发送闭环数据 10.001 Send closed loop data 10.001
####################################### 可用读取位移数据
#add = 1
#B3= 5 #//读取电压数据
#ch = 0
#ReadVolt = 0
#ReadVolt = readVoltOrMove(add,B3,ch)
#sleep(0.05)
#pinrt(ReadVolt)
####################################### 可用读取位移数据


####################################### 可用读取位移数据
#add = 1
#B3= 6 #//读取位移数据
#ch = 0
#ReadMove = 0
#ReadMove = readVoltOrMove(add,B3,ch)
#pinrt(ReadMove)
#######################################
sleep(0.02)
########################## 设置模拟 读取数字或模拟状态
add = 1
B3 = 20
FlagOpenOrClosed = 0x41 #A 0x41 D 0x44
Ch = 0
sentDigitalorAnalog(add,B3,FlagOpenOrClosed,Ch)
sleep(0.02)
B3Raed = 21
FlagOpenOrclose = ReadDigitalorAnalog(add,B3Raed,Ch)
print(FlagOpenOrclose)


sleep(0.02)
########################## 设置数字 读取数字或模拟状态
add = 1
B3 = 20
FlagOpenOrClosed = 0x44 #A 0x41 D 0x44
Ch = 0
sentDigitalorAnalog(add,B3,FlagOpenOrClosed,Ch)
sleep(0.02)
B3Raed = 21
FlagOpenOrclose = ReadDigitalorAnalog(add,B3Raed,Ch)
print(FlagOpenOrclose)



#sendTest(Usart)


# 创建一个线程用来接收串口数据 Create a thread to receive serial port data
# thread_uart = myThread(Usart)
# thread_uart.start()

# while True:
#         # 定时发送数据 Timed data
#         txbuf = "hello world"
#         len = uart_send_data(Usart, txbuf)
#         print("send len: ", len)
#         sleep(1)

#关闭开启的串口 Disable the enabled serial port
port_close() #20230505


