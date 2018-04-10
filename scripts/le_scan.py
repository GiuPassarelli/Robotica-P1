#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

import numpy as np

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def scaneou(dado):
	print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
	print("Leituras:")
	for i in range(len(np.array(dado.ranges).round(decimals=2))):
		if np.array(dado.ranges).round(decimals=2)[i]!=0 and np.array(dado.ranges).round(decimals=2)[i]<0.2:
			print(i)
			if i<45:
				velocidade = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, -0.4))
			elif i>315:
				velocidade = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0.4))
			elif i<180:
				velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, -0.4))
			elif i>180:
				velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0.4))
			velocidade_saida.publish(velocidade)
			return 0
	#print("Intensities")
	#print(np.array(dado.intensities).round(decimals=2))
		else:
			return 1


	


if __name__=="__main__":

	rospy.init_node("le_scan")

	velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	while not rospy.is_shutdown():
		print("Oeee")
		#(linha reta, )
		velocidade = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0))
		velocidade_saida.publish(velocidade)
		rospy.sleep(2)


