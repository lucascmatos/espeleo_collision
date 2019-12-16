#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

from std_msgs .msg import Bool, Int32, Float32
from array import array

angMax=0
angMin=0
count=0
#Menor valor de distancia
dist_min = 0
#Maior valor de distancia
dist_max = 0
#Vetor de pontos de distancia
vet = []
global i
vet_ponts = []
ponto_colisao = LaserScan()
#Puxa valores do arquivo de parametros do laser
model = rospy.get_param("model")
a = rospy.get_param("a")
d_min = rospy.get_param("d_min")
d_max = rospy.get_param("d_max")
#Seleciona os paremtros de distancia minima e max para o modelo em 2d
if model == "2D":
    dist_min = d_min
    dist_max = d_max
#_________________________________________
def callback(data):
   global ponto_colisao
   global vet
    #Pega as distancias medidas do topico
   vet = list(data.ranges)
   print len(data.ranges)
   #Varre o vetor de pontos
   for i in range(len(vet)):
        ponto_colisao = data
        #Checa se os pontos entao no meu range de colisao
        if dist_min<vet[i]<dist_max:
            vet_ponts.append(vet[i])
        else :
            vet_ponts.append(data.range_min)
   ponto_colisao.ranges =  list(vet_ponts)
def deteccao_colisao():
   global check
   rospy.init_node("dados_laser",anonymous = True)#incia no do pacote
   rospy.Subscriber("/scan",LaserScan,callback)#subscreve no no do laser
   pub = rospy.Publisher("colisao",LaserScan,queue_size=10)
   rate = rospy.Rate(10)
   while not rospy.is_shutdown():
        # A distancia do ponto mais proximo de colisao
        pub.publish(ponto_colisao)
   rate.sleep


if __name__== '__main__':
	try:
		deteccao_colisao()
	except rospy.ROSInterruptException:
		pass
