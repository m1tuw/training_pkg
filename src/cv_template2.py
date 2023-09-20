#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist, Point # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy



class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		#Suscribrirse a la camara
		self.Sub_Cam = rospy.Subscriber("/duckiebot/camera_node/image/raw", Image, self.procesar_img)
        #Publicar imagen(es)
		self.pub_img = rospy.Publisher("duckiebot_patos", Image, queue_size = 1)
		self.pub_img2 = rospy.Publisher("mascara", Image, queue_size = 1)


	def procesar_img(self, msg):
		#Transformar Mensaje a Imagen
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(msg, "bgr8")

		#Espacio de color

			#cv2.COLOR_RGB2HSV
			#cv2.COLOR_RGB2GRAY
			#cv2.COLOR_RGB2BGR
			#cv2.COLOR_BGR2HSV
			#cv2.COLOR_BGR2GRAY
			#cv2.COLOR_BGR2RGB

		image_out = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 

		#Definir rangos para la mascara
				
                n = 2
		if n ==0:
			lower_limit = np.array([25,100,130])
			upper_limit =np.array([35,255,255])
			tamanopato= 3.38
                        
                elif n ==2:
 			lower_limit=np.array([100,220,0])
			upper_limit=np.array([114,255,255])
			tamanopato= 6.3

		#Mascara
		mask = cv2.inRange(image_out, lower_limit, upper_limit)
		image_out = cv2.bitwise_and(image, image, mask=mask)
		
		sizey = len(mask)
		sizex = len(mask[0])

		# Operaciones morfolgicas, normalmente se utiliza para "limpiar" la mascara
		kernel = np.ones((5 , 5), np.uint8)
		img_erode = cv2.erode(mask, kernel, iterations=2) #Erosion
		img_dilate = cv2.dilate(img_erode, kernel, iterations=3) #Dilatar 	
		df=101.85916357881302
		

		# Definir blobs
		_,contours, hierarchy = cv2.findContours(img_dilate,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		for cnt in contours:
                        AREA = cv2.contourArea(cnt)
			if AREA>300: #Filtrar por tamano de obs
				x,y,w,h = cv2.boundingRect(cnt)
				dp=(tamanopato*df)/h
				print(dp)
				cv2.rectangle(image, (x,y), (x+w,y+h), (0,0,255), 2)
			else:
				None

		# Publicar imagen final
		msg = bridge.cv2_to_imgmsg(image, "bgr8")
		maskpub = bridge.cv2_to_imgmsg(image_out, "bgr8")
		self.pub_img.publish(msg)
		self.pub_img2.publish(maskpub)

def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
