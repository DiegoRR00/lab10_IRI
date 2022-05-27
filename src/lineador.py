#!/usr/bin/env python
from curses import panel
from gettext import find
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class line_follower:
    def __init__(self):
        self.img = np.array([])
        self.w = 0
        self.h = 0
        self.bridge = CvBridge()
        self.dt = 0.1
        self.max_v = 0.3
        rospy.init_node("line_follower")

        rospy.Subscriber('/video_source/raw',Image,self.source_callback)

        self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.debug_msg = rospy.Publisher('/ojos',Image,queue_size=10)

        self.t1 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback)
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop)

    def source_callback(self,msg):
        try:
            self.w = msg.width
            self.h = msg.height
            self.img = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        except:
            pass
        
    def getDistance(self,x1,y1,x2,y2):
        return np.sqrt((x1-x2)**2+(y1-y2)**2)
    def getAngle(self,x1,y1,x2,y2):
        return np.arctan2(y1-y2,x1-x2)
    def timer_callback(self,time):
        omega = 0
        v = 0
        negro = np.zeros((5,5),np.uint8)
        continuar = False
        try:
            imagen_resize = cv2.resize(self.img,None,fx=0.3,fy=0.3)
            continuar = True
        except:
            print("la vida es trsiteza")
            pass
        if continuar:
            img_gray = cv2.cvtColor(imagen_resize,cv2.COLOR_BGR2GRAY)
            img_gaus = cv2.GaussianBlur(img_gray,(5,5),cv2.BORDER_DEFAULT)
            imagen_recortada = img_gaus[int(img_gaus.shape[0]/3*2):int(img_gaus.shape[0])-1,int(img_gaus.shape[1]/5):int(img_gaus.shape[1]/5*4)-1]
            #img_bordes = cv2.Canny(imagen_recortada, 100, 170, apertureSize = 3)
            img_bordes = cv2.Canny(imagen_recortada, 10, 100, apertureSize = 3)
            k = np.ones((3,3),np.uint8)
            #img_bordes = cv2.dilate(img_bordes,k,iterations = 1)
            negro = np.zeros(img_bordes.shape,np.uint8)
            
            #lines = cv2.HoughLines(img_bordes, 1.2, np.pi / 180, 30,0,0)
            #try:
                #lines = cv2.HoughLines(img_bordes, 1.2, np.pi / 180, 30,0,0)
                #lines = cv2.HoughLines(img_bordes, 1.2, np.pi / 180, 30,0,0)
            lines = cv2.HoughLinesP(img_bordes,0.1,np.pi/180*1.5,3,100,3)
        
            print("oklol")
            
            # Iterate through each line and convert it to the format
            '''
            for i in range(len(lines)):
                rho, theta = lines[i,0,:]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                
                cv2.line(negro, (x1, y1), (x2, y2), (255, 255, 255), 1)
            
            print("lol")
            print(len(lines))
            print(len(lines[0]))
            for l in lines:
                #print(l)
                x1,y1,x2,y2 = l[0][:]
                #print(x1,y1,x2,y2)
                cv2.line(negro,(x1,y1),(x2,y2),(255,255,255),1)
            '''
            tamano = []
            continuar = False
            try:
                for x1,y1,x2,y2 in lines[0]:
                    tamano.append(np.sqrt((x1-x2)**2+(y1-y2)**2))
                tamano.sort()
                continuar = True
            except:
                print("No lineas")
            if continuar:
                print("linas",lines)
                vel = 0.1
                dtheta = 0
                if len(lines) > 20:
                    vel = 0
                    dtheta = 0
                    for l in lines:
                        #print(l)
                        x1,y1,x2,y2 = l[0][:]
                        #print(x1,y1,x2,y2)
                        cv2.line(negro,(x1,y1),(x2,y2),(255,255,255),1)
                else:
                    idx = tamano.index(min(tamano))
                    x1,y1,x2,y2 = lines[0][idx]
                    cv2.line(negro,(x1,y1),(x2,y2),(255,255,255),1)
                    puntoMedio = [img_bordes.shape[0]-1,img_bordes.shape[1]/2]
                    
                    print("linea",(x1,y1),(x2,y2))
                    distancias = [self.getDistance(x1,y1,puntoMedio[0],puntoMedio[1]),self.getDistance(x2,y2,puntoMedio[0],puntoMedio[1])]
                    print("Puntomedio y distancias",puntoMedio,distancias)
                    if distancias[0] > distancias[1]:
                        puntoObjetivo = [x1,y1]
                    else:
                        puntoObjetivo = [x2,y2]

                    print("pObj",puntoObjetivo)
                    dtheta = np.arctan2(puntoObjetivo[1]-puntoMedio[1],puntoObjetivo[0]-puntoMedio[0]) + np.pi/2#self.getAngle(puntoObjetivo[0],puntoObjetivo[1],puntoObjetivo[0],puntoMedio[1])
                    print("dt",dtheta)
                msg = Twist()
                msg.linear.x = vel
                msg.linear.y = 0
                msg.linear.z = 0
                msg.angular.x = 0
                msg.angular.y = 0
                msg.angular.z = -dtheta * 0.1
                #publicar
                self.twist_publisher.publish(msg)


            msg_img = Image()
            msg_img = self.bridge.cv2_to_imgmsg(negro)
            self.debug_msg.publish(msg_img)
        

        

    def stop(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        #publicar
        self.twist_publisher.publish(msg)
        print("Muerte y destruccion o shutdown")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    lineator = line_follower()
    try:
        lineator.run()
    except:
        pass

