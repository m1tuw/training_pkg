#!/usr/bin/env python

import message_filters
import rospy #importar ros para python
from std_msgs.msg import String, Int32 #importa mensajes de ROS tipo String y Int32
from sensor_msgs.msg import Joy # impor mensaje tipo Joy
from geometry_msgs.msg import Twist, Point # importar mensajes de ROS tipo geometry / Twist
from duckietown_msgs.msg import Twist2DStamped 


class Template(object):
    def __init__(self, args):
        super(Template, self).__init__()
        self.args = args
        #sucribir a joy 
        self.sub_joy = message_filters.Subscriber("/duckiebot/possible_cmd",Twist2DStamped)
        self.sub_dist = message_filters.Subscriber("distancia", Point)
        #publicar la intrucciones del control en possible_cmd
        self.publi = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size = 10)
        self.twist = Twist2DStamped()

	ts = message_filters.TimeSynchronizer([self.sub_dist, self.sub_joy], 10)
	ts.registerCallback(self.callback)

    def callback(self,msg,dist_node):

	dist = dist_node.x
	if dist < 15:
		self.twist.v = 0
	else:
		pass
                
        self.publi.publish(self.twist)  
        
    #def publicar(self, msg):
        #self.publi.publish(msg)
	



def main():
    rospy.init_node('test') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

    #objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
    main()
