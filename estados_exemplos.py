#! /usr/bin/env python
# -*- coding:utf-8 -*-

__author__ = ["Rachel P. B. Moraes", "Igor Montagner", "Fabio Miranda"]


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

import cormodule_caixa
import seguemodule_soldadinho


from sensor_msgs.msg import LaserScan

bridge = CvBridge()

cv_image = None

# Variáveis para permitir que o roda_todo_frame troque dados com a máquina de estados
media = []
centro = []
area = 0.0
perigoso = 'semperigo'



tolerancia_x = 50
tolerancia_y = 20
ang_speed = 0.2
area_ideal = 60000 # área da distancia ideal do contorno - note que varia com a resolução da câmera
tolerancia_area = 20000

# Atraso máximo permitido entre a imagem sair do Turbletbot3 e chegar no laptop do aluno
atraso = 1.5
check_delay = False # Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados


def scaneou(dado):
    global leituras
    global perigoso

    leituras = dado
    perigoso = 'semperigo'
    print("Faixa valida: ", leituras.range_min , " - ", leituras.range_max )
    print("Leituras:")
    for i in range(len(np.array(leituras.ranges).round(decimals=2))):
        if np.array(leituras.ranges).round(decimals=2)[i]!=0 and np.array(leituras.ranges).round(decimals=2)[i]<0.2:
            perigoso = 'perigo'
            

def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global media
    global centro
    global area
    global mediaS
    global centroS
    global areaS

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime
    delay = lag.secs
    if delay > atraso and check_delay==True:
        return 
    try:
        antes = time.clock()
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        media, centro, area = cormodule_caixa.identifica_cor(cv_image)
        mediaS, centroS, areaS = seguemodule_soldadinho.identifica_frame(cv_image)
        
        depois = time.clock()
        cv2.imshow("Camera", cv_image)
    except CvBridgeError as e:
        print('ex', e)
    





## Classes - estados

class Perigo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['perigo', 'semperigo'])

    def execute(self, userdata):
        global velocidade_saida
        global perigoso

        if perigoso == 'semperigo':
            return 'semperigo'
            print(leituras)
        for i in range(len(np.array(leituras.ranges).round(decimals=2))):
            if np.array(leituras.ranges).round(decimals=2)[i]!=0 and np.array(leituras.ranges).round(decimals=2)[i]<0.2:
                perigoso = 'perigo'
                if i<=45:
                    velocidade = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, -0.4))
                elif i>=315:
                    velocidade = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0.4))
                elif i<180:
                    velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, -0.4))
                elif i>=180:
                    velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0.4))
                velocidade_saida.publish(velocidade) 
                return 'perigo' 



        


class Girando(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhou', 'girando','perigo'])

    def execute(self, userdata):
        global velocidade_saida
        global perigoso

        if perigoso == 'perigo':
            return 'perigo'
        else:
            if (media is None or len(media)==0) and (mediaS is None or len(mediaS)==0):
                vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
                velocidade_saida.publish(vel)
                return 'girando'

            if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x) or math.fabs(mediaS[0]) > math.fabs(centro[0] + tolerancia_x):
                vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ang_speed))
                velocidade_saida.publish(vel)
                return 'girando'
            if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x) or math.fabs(mediaS[0]) < math.fabs(centro[0] - tolerancia_x):
                vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, ang_speed))
                velocidade_saida.publish(vel)
                return 'girando'
            else:
                vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                velocidade_saida.publish(vel)
                return 'alinhou'


class Centralizado(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['alinhando', 'alinhado','perigo'])

    def execute(self, userdata):
        global velocidade_saida
        global perigoso
        
        if perigoso == 'perigo':
            return 'perigo'
        else:
            if media is None:
                return 'alinhou'
            if  math.fabs(media[0]) > math.fabs(centro[0] + tolerancia_x):
                vel = Twist(Vector3(-0.5, 0, 0), Vector3(0, 0, 0))
                velocidade_saida.publish(vel)
                return 'alinhando'
            if math.fabs(media[0]) < math.fabs(centro[0] - tolerancia_x):
                vel = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0))
                velocidade_saida.publish(vel)
                return 'alinhando'
            else:
                vel = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0))
                velocidade_saida.publish(vel)
                return 'alinhado'



# main
def main():
    global velocidade_saida
    global velocidade
    global perigoso
    global buffer
    rospy.init_node('estados_exemplos')

    # Para usar a webcam 
    #recebedor = rospy.Subscriber("/cv_camera/image_raw/compressed", CompressedImage, roda_todo_frame, queue_size=1, buff_size = 2**24)
    recebedor = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, roda_todo_frame, queue_size=10, buff_size = 2**24)
    roda_laser = rospy.Subscriber("/scan", LaserScan, scaneou)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['terminei'])

    # Open the container
    with sm:
        # Add states to the container
        #smach.StateMachine.add('LONGE', Longe(), 
        #                       transitions={'ainda_longe':'ANDANDO', 
        #                                    'perto':'terminei'})
        #smach.StateMachine.add('ANDANDO', Andando(), 
        #                       transitions={'ainda_longe':'LONGE'})
        smach.StateMachine.add('PERIGO', Perigo(),
                                transitions={'perigo': 'PERIGO',
                                'semperigo':'GIRANDO'})
        smach.StateMachine.add('GIRANDO', Girando(),
                                transitions={'girando': 'GIRANDO',
                                'alinhou':'CENTRO', 'perigo':'PERIGO'})
        smach.StateMachine.add('CENTRO', Centralizado(),
                                transitions={'alinhando': 'GIRANDO',
                                'alinhado':'CENTRO','perigo':'PERIGO'})


    # Execute SMACH plan
    outcome = sm.execute()
    #rospy.spin()


if __name__ == '__main__':
    main()