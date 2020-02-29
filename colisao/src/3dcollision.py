#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
from array import array
from math import atan2,degrees
from std_msgs .msg import Bool, Int32
#Vetores de pontos
vetx = []
vety = []
vetz = []
# soma dos quadrados dos pontos
sum_root_2d = 0
sum_root_3d = 0
#Variavel para contagem
i = 0
# contador de nivel
j = 0
# incializa variavel nivel
nivel = 1
#controle de nivel
controle_nivel = 0
 # Controle do for
cont = 0
# angulo entre pontos
angle = 0
# Vetor de comparacao entre variaveis
comp = []
# Comparacao entre Z
compz = []
# Pontos em Z
ponto_z = 0
# Filtra dados
pontos = 0
# Classifica como colisao e publica
colisao = 0
#Variavel de teste
pontos_colisao = 0
#Puxa valores do arquivo de parametros do laser
model = rospy.get_param("model")
a = rospy.get_param("a")
d_min = rospy.get_param("d_min") #Usado para filtrar ruidos
d_max = rospy.get_param("d_max")
#Seleciona os paremtros de distancia minima e max para o modelo em 3d
if model == "3D":
  angulo = a
  dist_min = d_min
  dist_max = d_max
#_________________________________________
def callback(data):
    global i,j,vetx,vety,vetz,cont,controle_nivel,nivel,angle,pontos,colisao,pontos_colisao,compz,comp
    for i in range (0,len(data.points)):
        #Vetores que salvam os valores de x,y,z da nuvem de pontos.
        vetx.append(float(data.points[i].x))
        vety.append(float(data.points[i].y))
        vetz.append(float(data.points[i].z))
        #Distancia em 2d
        sum_root_2d = (vetx[i]**2 + vety[i]**2)**0.5
        #Distancia em 3d
        sum_root_3d = (vetx[i]**2 + vety[i]**2 + vetz[i]**2)**0.5
        # Controle de nivel dos 16 ponto em sequencia para varredura.
        if j >15:
            nivel = nivel +1
            j = 0
		#identifica ponto como proximo dado um certo valor de distancia.
        if  sum_root_3d>dist_min and sum_root_3d<dist_max or cont == 1:
            controle_nivel = (nivel*16) -1            
            if sum_root_2d != 0.0 :
                comp.append(sum_root_2d)
            if len(comp) == 2:
                ponto_z =  len(vetz) - 1
                compz.append(float(vetz[ponto_z]))
            if len(compz)>2:                
                angle  = atan2((compz[1]-compz[0]),(comp[1]-comp[0]))
                comp.pop(0)
                compz.pop(0)
                angle = degrees(angle)
            if abs(angle) >= angulo and abs(angle) != 180:
                pontos = pontos +1
			#Filtra se ele detecta apenas 1 ponto de colisao.
			#se houver mais de um ponto significa que a risco de colisao.
            if pontos >2:
                colisao = True
                #Local onde ha perigo de colisao em relacao ao robo.
                if 12258<i<20480:
					#Colisao Frontal.
                    pontos_colisao = 1
                if 8192<i<12258:
					#Colisao da quina frontal esquerda.
					pontos_colisao = 2
                if 20480<i<24576:
					#Colisao da quina frontal direita.
                    pontos_colisao = 3
                if 4096<i<8192:
					#Colisao da quina traseira esquerda.
                    pontos_colisao = 4
                if 24576<i<28672:
					#Colisao da quina traseira direita.
                    pontos_colisao = 5
                if 28672<i<4096:
					# Colisao traseira.
                    pontos_colisao = 6
			#Se nao entao variavel de controle de colisao permanece em zero.
            else:
                pontos_colisao = 0
                colisao = False
			#Enquanto cont for igual a 1 ele continua varrendo meus 16 pontos.
            if i<(controle_nivel):
                cont = 1
			#Se for igual a zero todos os valores recebem zero para poder calcular os proximos pontos.
            if i == (controle_nivel):
                cont = 0
                pontos = 0
                comp = []
                compz = []
        #Incrimento de J para controle de nivel.
        j = j+1
        #Incremento de i para varrer os pontos.
        i = i+1
	    	#Fim do loop zero os pontos.
        if i ==len(data.points):
            i = 0
            nivel = 0
            vetx=[]
            vety=[]
            vetz=[]

def getdata():
	global colisao
	rospy.init_node("detect_colision",anonymous=True)
	rospy.Subscriber("/velodyne_points",PointCloud,callback)
	#Publica no topico colisao true se detectar colisao e false se nao.
	pub = rospy.Publisher("colisao",Bool,queue_size = 10)
	#Publica em qual setor do robo ocorreu a colisao.
	col = rospy.Publisher("pontos",Int32,queue_size = 10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.publish(colisao)
		col.publish(pontos_colisao)
	   	rate.sleep()
if __name__== '__main__':
	try:
		getdata()
	except rospy.ROSInterruptException:
		pass
