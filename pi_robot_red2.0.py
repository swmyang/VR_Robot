#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
树莓派机器人Red物体检测控制程序
Version：1.1
Date：2019-7-19
Author：Wu Fang & Kezhen Zhang
"""

"""
程序说明
流程：
(1)目标检测与识别
物体目标搜索策略
(2)摄像机标定？
(3)目标定位与抓取  
目标定位阶段,根据实际的实验环境,制定合理的目标搜索策略,然后对目标物体相对机器人的位置进行等价分区,利用测距算法,
在此基础上加入误差矫正提高定位的准确性。物体抓取,执行程序的流程,通过图像中物体大小比较得到位置信息,
调整机器人姿态,最后控制机械手运动抓取物体。

函数说明 
move_position()  --控制机器人到标定的抓取位子
forward（）--前进
back()  --后退
stop()  --停止
turn_left --左转
turn_right --右转

grasp() --抓取
put() --放
rest() --复位   
参数
lower_white --hsv 颜色模型下限值
（x,y,w,h）-- 图像中物体的(x, y)左上点坐标，(x+w，y+h）是物体的右下点坐标,w，h是物体的宽和高

"""
import numpy as np
import cv2 
import time
import serial
import string
import binascii

#控制机器人运动函数
# 前进
def forward():
    d=bytes.fromhex('FF 01 05 FF 00 01 00 FE')
    for i in range(5):
        s.write(d)
# 后退
def back():
    d=bytes.fromhex('FF 01 05 FF 00 02 00 FE')
    for i in range(5):
        s.write(d)
# 停止
def stop():
    d=bytes.fromhex('FF 01 05 FF 00 00 00 FE')
    for i in range(5):
        s.write(d)
# 左转
def turn_left():
    d=bytes.fromhex('FF 01 05 FF 00 03 00 FE')
    for i in range(5):
        s.write(d)
# 右转
def turn_right():
    d=bytes.fromhex('FF 01 05 FF 00 04 00 FE')
    for i in range(5):
        s.write(d)
#根据不同阶段，配置三种不同速度，根据实际调试情况改变速度参数
# 调速度--慢
def speed_V1():
    d=bytes.fromhex('FF 01 05 FF 00 07 00 FE') # v1
    for i in range(5):
        s.write(d)
# 调速度--快
def speed_V2():
    d=bytes.fromhex('FF 01 05 FF 00 09 00 FE') # v1
    for i in range(5):
        s.write(d)
# 调速度--较快
def speed_V3():
    d=bytes.fromhex('FF 01 05 FF 00 0B 00 FE') # v1
    for i in range(5):
        s.write(d)
#控制机器人到标定的抓取
# 抓取物品
def grasp():
    d=bytes.fromhex('FF 03 0F 23 30 30 33 50 32 30 35 30 54 31 30 30 30 21')
    for i in range(5):
        s.write(d)
    d=bytes.fromhex('FF 03 0F 23 30 30 33 50 32 30 35 30 54 31 30 30 30 21')
    for i in range(5):
        s.write(d)
    time.sleep(0.01)#03duoji
    d=bytes.fromhex('FF 03 0F 23 30 30 34 50 31 30 30 30 54 31 30 30 30 21')
    for i in range(2):
        s.write(d)
    time.sleep(2)
    d=bytes.fromhex('FF 03 0F 23 30 30 30 50 31 37 30 30 54 31 30 30 30 21')
    for i in range(2):
        s.write(d)
    d=bytes.fromhex('FF 03 0F 23 30 30 31 50 30 35 30 30 54 35 30 30 30 21')
    for i in range(5):
        s.write(d)
    d=bytes.fromhex('FF 03 0F 23 30 30 32 50 31 35 30 30 54 31 30 30 30 21')
    for i in range(2):
        s.write(d)
    time.sleep(6)
    d=bytes.fromhex('FF 03 0F 23 30 30 35 50 31 32 30 30 54 31 30 30 30 21')
    for i in range(2):
        s.write(d)
        
def put():
    time.sleep(2)
    d=bytes.fromhex('FF 03 0F 23 30 30 31 50 31 35 30 30 54 35 30 30 30 21')
    for i in range(5):
        s.write(d)
    d=bytes.fromhex('FF 03 0F 23 30 30 33 50 31 35 30 30 54 31 30 30 30 21')
    for i in range(5):
        s.write(d)
    d=bytes.fromhex('FF 03 0F 23 30 30 34 50 30 39 30 30 54 31 30 30 30 21')
    for i in range(5):
        s.write(d)
    time.sleep(2)
    d=bytes.fromhex('FF 03 0F 23 30 30 30 50 31 35 30 30 54 31 30 30 30 21')
    for i in range(5):
        s.write(d)
    d=bytes.fromhex('FF 03 0F 23 30 30 32 50 31 35 30 30 54 31 30 30 30 21')
    for i in range(5):
        s.write(d)
    time.sleep(6)
    d=bytes.fromhex('FF 03 0F 23 30 30 35 50 31 35 30 30 54 31 30 30 30 21')
    for i in range(5):
        s.write(d)

def rest():
    d=bytes.fromhex('FF 03 0F 23 30 30 30 50 31 35 30 30 54 35 30 30 30 21')
    for i in range(5):
        s.write(d)
    d=bytes.fromhex('FF 03 0F 23 30 30 31 50 31 35 30 30 54 35 30 30 30 21')
    for i in range(5):
        s.write(d)
    d=bytes.fromhex('FF 03 0F 23 30 30 32 50 31 35 30 30 54 35 30 30 30 21')
    for i in range(5):
        s.write(d)
    d=bytes.fromhex('FF 03 0F 23 30 30 33 50 31 35 30 30 54 35 30 30 30 21')
    for i in range(5):
        s.write(d)
    d=bytes.fromhex('FF 03 0F 23 30 30 34 50 31 35 30 30 54 35 30 30 30 21')
    for i in range(5):
        s.write(d)
    d=bytes.fromhex('FF 03 0F 23 30 30 35 50 31 35 30 30 54 35 30 30 30 21')
    for i in range(5):
        s.write(d)    
       
#控制机器人到标定的抓取位子
def move_position():
     #控制机器人到标定的抓取位置和角度，需要反复调试确定参数
    #根据图像中物体中心点坐标控制机器人左转或右转到标定的抓取角度
    if x+w/2 < xcbd-10:  
        turn_left()         # 串口发送左转指令
        time.sleep(0.01)
    
    elif x+w/2 > xcbd-10 and x+w/2 < xcbd+10:
   #     stop()
     #根据图像中物体大小控制机器人前进或后退到标定的抓取位置，wbd 和 hbd 为抓取位置时标定值
        if w < wbd-50:  # 机器人比较远时，前进速度为 较快 v3，参数50，30，10可调（根据实际机器人初始位置情况）
            speed_V3()     
            forward()  # 串口发送前进指令
        elif w > wbd-50 and w < wbd - 30:
            speed_V2()         # 机器人离物品距离变短时，前进速度降为v2
            forward()
        elif w > wbd-30 and w < wbd - 10:
            speed_V1()         # 机器人靠近物品时，前进速度降为v1
            forward()
        elif w > wbd-10 and w < wbd + 10:
            stop()    # 串口发送停止指令
            grasp()    # 发送抓取指令
            put()   
        else:
            back()              
            
    else:
        turn_right()        # 串口发送右转指令
        time.sleep(0.01)


#设定白色阈值  hsv 颜色模型值，可用中心红点对准物体获得标定 hsv 颜色值作为参数选择依据
#lower_white = np.array([32, 22, 29])#white
#lower_white = np.array([80, 40, 50])
lower_white = np.array([150, 120, 150])#red
upper_white = np.array([255, 255, 255])
#lower_white = np.array([110, 20, 120])#blue
#upper_white = np.array([255, 255, 255])

size=640,480

#打开摄像头
cap = cv2.VideoCapture(0)  
#等待两秒  
time.sleep(2)
#手动抓物体获得大小的标定值
wbd = 200      #190~210
hbd = 400       
xcbd = 340        #335~355
ycbd = 270      
#初始化串口，加代码
s = serial.Serial("/dev/ttyAMA0", 115200,timeout=1)#打开串口

rest()   
while(1): 	
    (ret,frame) = cap.read()
    if not ret:  
        print ('No Camera\n')
        break

    #转到白色的HSV空间
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    #红点标出图像的中心坐标
    cv2.circle(frame, (320,240), 10, (0, 0, 255), -1)
    #输出图片中心的hsv 值, 可作为标定颜色参考值
    #print("The hsv of the center is:" + str(hsv[320,240])) 
      
    #根据阈值构建蒙版掩膜  	 		
    mask_white = cv2.inRange(hsv,lower_white,upper_white)
    #腐蚀操作,去除噪点 
    mask_white = cv2.erode(mask_white, None, iterations=2)
    #膨胀操作，去除噪点  
    mask_white = cv2.dilate(mask_white, None, iterations=2)
    # 轮廓检测
    mask = mask_white
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] 
    # speed config
    speed_V1()         # v1
    # left旋转搜寻物体
    turn_left() 
    #如果存在物体轮廓  
    if len(cnts) > 0:  
        #找到面积最大的轮廓 ----目标物体，停止转动
        stop()
        c = max(cnts, key = cv2.contourArea)  
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)#img图片，(x, y)左上点坐标，(x+w，y+h）是矩阵的右下点坐标,w，h是矩阵的宽和高
        #控制机器人到标定的抓取位子
        move_position()
        print (x+w/2, y+h/2,w,h)    # 输出物体中心位置坐标，宽度和高度
        #print(hsv[int(x+w/2)-2, int(y+h/2)])  #输出物体中心位置hsv的值
   
    cv2.imshow('Frame', frame)
    cv2.imshow('Frame_hsv', hsv)
    cv2.imshow('Frame_mask', mask)
    if cv2.waitKey(5) == 27 or cv2.waitKey(5) == ord('q'):
        print('exit!\n')
        break

ser.close()
cap.release()
cv2.destroyAllWindows() 
