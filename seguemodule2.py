#! /usr/bin/env python
# -*- coding:utf-8 -*-


import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import smach
import smach_ros

MIN_MATCH_COUNT = 10
detector = cv2.xfeatures2d.SIFT_create()

FLANN_INDEX_KDITREE = 0
flannParam = dict (algorithm=FLANN_INDEX_KDITREE, tree=5)
flann = cv2.FlannBasedMatcher(flannParam,{})

trainImg = cv2.imread("cigarro.jpg", 0)

#cv2.imshow('teste',trainImg)  

trainKP,trainDesc = detector.detectAndCompute(trainImg, None)

#cv2.waitKey(0)

def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged

def identifica_frame(frame):



    media = [-1,-1]
    centro = (frame.shape[0]//2, frame.shape[1]//2)



    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # A gaussian blur to get rid of the noise in the image
    blur = cv2.GaussianBlur(gray,(5,5),0)
    #blur = gray
    # Detect the edges present in the image
    bordas = auto_canny(blur)
    # Obtains a version of the edges image where we can draw in color
    bordas_color = cv2.cvtColor(bordas, cv2.COLOR_GRAY2BGR)


    ret2,thresh = cv2.threshold(bordas,127,255,1)
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        if len(approx)==4:
            cv2.drawContours(bordas_color,[cnt],0,(0,0,255),-1)
    cv2.imshow('img',bordas_color)        



    #DETECTAR A RAPOSA:
    queryKP, queryDesc = detector.detectAndCompute(gray,None)
    matches = flann.knnMatch(queryDesc, trainDesc, k=2)

    goodMatch = []
    for m,n in matches:
        if m.distance < 0.75*n.distance:
            goodMatch.append(m)
    if len(goodMatch) > MIN_MATCH_COUNT:
        tp = []
        qp = []
        for m in goodMatch:
            tp.append(trainKP[m.trainIdx].pt)
            qp.append(queryKP[m.queryIdx].pt)
        tp,qp = np.float32((tp,qp))
        H,status = cv2.findHomography(tp,qp,cv2.RANSAC,3.0)
        h,w = trainImg.shape
        trainBorder = np.float32([[[0,0],[0,h-1],[w-1,h-1],[w-1,0]]])
        queryBorder = cv2.perspectiveTransform(trainBorder,H)
        cv2.polylines(bordas_color,[np.int32(queryBorder)],True,(0,255,0),5)
        print(queryBorder)
        media = [(queryBorder[0][0][0]+queryBorder[0][1][0]+queryBorder[0][2][0]+queryBorder[0][3][0])/4,(queryBorder[0][0][1]+queryBorder[0][1][1]+queryBorder[0][2][1]+queryBorder[0][3][1])/4]
    else:
        print "Not Enough match found- %d/%d" %(len(goodMatch),MIN_MATCH_COUNT)
        #media = [0,0] mudar para 200 e testar
        #centro = [0,0]
    cv2.imshow('result', bordas_color)


    

    return media,centro, 0

	