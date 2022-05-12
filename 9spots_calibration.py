# -*- coding: utf-8 -*-
"""
树莓派机器人9点标定程序
Version：1.0
Date：2019-7-22
Author：Wu Fang & Kezhen Zhang
"""

"""
程序说明
流程：

摄像机标定？
双击鼠标左键 获得 鼠标当前图像像素 位置坐标（x,y）

函数 
 draw_circle


   
参数
lower_white --hsv 颜色模型下限值
（x,y,w,h）-- 图像中物体的(x, y)左上点坐标，(x+w，y+h）是物体的右下点坐标,w，h是物体的宽和高

"""


import numpy as np
import cv2 
import time

#设定白色阈值  hsv 颜色模型值，可用中心红点对准物体获得标定 hsv 颜色值作为参数选择依据
#lower_white = np.array([32, 22, 29])
#lower_white = np.array([80, 40, 50])
lower_white = np.array([32, 22, 29])
upper_white = np.array([180, 255, 255])

size=640,480

#打开摄像头
cap = cv2.VideoCapture(0)  
#等待两秒  
time.sleep(2)
#手动抓物体获得大小的标定值
wbd = 120
hbd = 220
ix,iy=-1,-1
#mouse callback function
def draw_circle(event,x,y,flags,param):
    if event==cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(frame,(x,y),20,(0, 0, 255),-1)
        print(x,y)
    
while(1): 	
    (ret,frame) = cap.read()
    if not ret:  
        print ('No Camera\n')
        break
    #转到白色的HSV空间
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    #红点标出图像的中心坐标
    cv2.circle(frame, (320,240), 10, (0, 0, 255), -1)
    cv2.imshow('Frame', frame)
    cv2.setMouseCallback('Frame',draw_circle)

    if cv2.waitKey(5) == 27 or cv2.waitKey(5) == ord('q'):
        print('exit!\n')
        break
# 机械臂初始化
        
# 配置 home position
        
# Calibration points
default_cali_points = [[180,-120,135,0],[270,-120,135,0],
                       [180,120,135,0],[270,120,135,0],
                       [270,120,-5,0],[180,120,-5,0],
                       [180,-120,-5,0],[270,-120,-5,0]]
np_cali_points = np.array(default_cali_points)
arm_cord = np.column_stack((np_cali_points[:,0:3], np.ones(np_cali_points.shape[0]).T)).T
    

cap.release()
cv2.destroyAllWindows() 

