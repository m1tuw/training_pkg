#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 #importa mensajes de ROS tipo String y Int32
from sensor_msgs.msg import Joy # impor mensaje tipo Joy
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from duckietown_msgs.msg import Twist2DStamped 


class Template(object):
    def __init__(self, args):
        super(Template, self).__init__()
        self.args = args
        #sucribir a joy 
        self.sub = rospy.Subscriber("/duckiebot/joy" , Joy, self.callback)
        #publicar la intrucciones del control en possible_cmd
        self.twist = Twist2DStamped()
        self.publi = rospy.Publisher("/duckiebot/possible_cmd",Twist2DStamped, queue_size = 10) 


    #def publicar(self, msg):
        #self.publi.publish(msg)
	

    def callback(self,msg):
        a = msg.buttons[0]
	b = msg.buttons[1]
	x = msg.buttons[2]
	y = msg.buttons[3]

	lb, rb = msg.buttons[4], msg.buttons[5]

	rt = msg.axes[5]

	leftjoy_x = -msg.axes[0]
	leftjoy_y = msg.axes[1]

	rightjoy_x = -msg.axes[3]
	rightjoy_y = msg.axes[4]

	deadzone = 0.2

        if rt < 0:
           maxomega = 5
           maxspeed = 0.3
        else:
	   maxomega = 10
	   maxspeed = 1
	
	if abs(leftjoy_x) > deadzone:
		self.twist.omega = leftjoy_x * maxomega
	else:
		self.twist.omega = 0

	if abs(rightjoy_y) > deadzone:
		self.twist.v = rightjoy_y * maxspeed
	else:
		self.twist.v = 0
	

# panic button
	if rb == 1 and lb == 1:
		self.twist.omega = 0
		self.twist.v = 0
        
        self.publi.publish(self.twist)
        




def main():
    rospy.init_node('test') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

    #objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
    main()
