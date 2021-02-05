#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud
from array import array
from math import atan2,degrees,acos,sqrt
from std_msgs .msg import Int32,String,Int16
import numpy as np
from copy import copy


########################################################################
#Puxa valores do arquivo de parametros do laser
model = rospy.get_param("model")
theta_limit = rospy.get_param("theta_limit")
d2D_yellow = rospy.get_param("d2D_yellow") #Usado para filtrar ruidos
d2D_red = rospy.get_param("d2D_red")
pairs_of_points  = rospy.get_param("pairs_of_points")
# Variavel para manter como colisao enquanto houver qualquer ponto que seja classificado como perigoso
check_collision = 0
#Seleciona os paremtros de distancia minima e max para o modelo em 3d
if model == "3D":
  angulo = theta_limit
  dist_min = d2D_yellow
  dist_max = d2D_red
  p = pairs_of_points
########################################################################
# Classifica como colisao e publica
global colisao
colisao = 0
#Colisao antigo
global old_colisao
old_colisao = 0
#Variavel da os setores de colisao
global pontos_colisao
pontos_colisao = 0
#Variavel para publicar pontos onde ocorreu colisao
colisao_visual =PointCloud()

def callback(data):
    global colisao,old_colisao,pontos_colisao,colisao_visual
    #variavel para salvar pontos onde ocorre colisao da nuvem
    aux_colisao_visual = [] 
    colisao_visual = copy(data)
    colisao_visual.points = []
    #Vetores de pontos
    vetx = np.array([])
    vety = np.array([])
    vetz = np.array([])
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
    comp = np.array([])
    # Comparacao entre Z
    compz = np.array([])
    # Pontos em Z
    ponto_z = 0
    # Filtra dados
    pontos = 0
    #variavel que sinaliza colisao
    check_collision = 0
    #variavel para salvar a distancia em z
    dz = 0
    #variavel para salvar a distancia no plano
    dp = 0
    #varial que altera a cor do led
    green = 0
    yellow = 0
    red = 0
    #variavel para salvar a que distnacia do robo um angulo maior que 30 foi detectado
    d2D = 0
    #Altura do ponto
    height = 0
    #Inicio do programa
    for i in range (0,len(data.points)):
        #Vetores que salvam os valores de x,y,z da nuvem de pontos.
        vetx = np.append(vetx, float(data.points[i].x))
        vety = np.append(vety, float(data.points[i].y))
        vetz = np.append(vetz, float(data.points[i].z))
        #Distancia em 3d
        sum_root_3d = sqrt((vetx[i]**2 + vety[i]**2 + vetz[i]**2))
        # Controle de nivel dos 16 ponto em sequencia para varredura.
        if j >15:
            nivel = nivel +1
            j = 0
	#identifica pontos proximos.
        if  (sum_root_3d<2.00 or cont == 1):
            #Distancia em 2d
            sum_root_2d = sqrt((vetx[i]**2 + vety[i]**2))
            controle_nivel = (nivel*16) -1
            if (sum_root_2d != 0.0 ):
		comp = np.append(comp, sum_root_2d)
		compz = np.append(compz, float(vetz[i]))
            if (len(comp) == 2):
                #calcula angulo de inclinacao do objeto
                if (comp[0] == comp[1]):
                    dz = sqrt((compz[1]- compz[0])**2)
                    angle = degrees(atan2(dz,abs(comp[1])))
                    if angle >30:
                        height = compz[1] #Altura do laser em relacao ao ponto
                        d2D =  comp[1] - 0.25
                        pontos = pontos+1
                else:
                    dz = sqrt((compz[1]- compz[0])**2)
                    dp = sqrt((comp[1]- comp[0])**2)
                    angle = degrees(atan2(dz,dp))
                    if angle >30:
                        height = compz[1] #Altura do laser em relacao ao ponto
                        d2D =  comp[1] - 0.25
                        pontos = pontos+1                        
                comp = np.delete(comp,0)
                compz = np.delete(compz,0)
            # se o angulo for maior que um valor limite add a uma variavel contadora
            if (angle >= theta_limit and d2D<dist_min and height<-0.15 and pontos>=p): # height altura do menor ponto
                yellow = 1
            
                #Variavel que salva os pontos de colisao
                #aux_colisao_visual.append(data.points[i])
			#se houver mais de p pontos com colisao risco de colisao que o obstaculo representa perigo.
            if (angle >= theta_limit and d2D<dist_max and height<-0.15 and pontos>=p):# height altura do menor ponto 
                red = 2    
                             
                #Variavel que salva os pontos de colisao
                aux_colisao_visual.append(data.points[i])
                #Local onde ha perigo de colisao em relacao ao robo.
                if (12258<i<20480):
					#Colisao Frontal.
                    pontos_colisao = 1
                elif (8192<i<12258):
					#Colisao da quina frontal esquerda.
					pontos_colisao = 2
                elif (20480<i<24576):
					#Colisao da quina frontal direita.
                    pontos_colisao = 3
                elif (4096<i<8192):
					#Colisao da quina traseira esquerda.
                    pontos_colisao = 4
                elif (24576<i<28672):
					#Colisao da quina traseira direita.
                    pontos_colisao = 5
                elif (28672<i<4096):
					# Colisao traseira.
                    pontos_colisao = 6
            else:
                pontos_colisao = 0
                check_collision = 0
            # Publica a variavel colisao
            	#Enquanto cont for igual a 1 ele continua varrendo meus 16 pontos.
            if (i<(controle_nivel)):
                cont = 1
	        #Zera os valores pare calcular novo risco de colisao
            if (i == (controle_nivel)):
                cont = 0
                pontos = 0
                comp = []
                compz = []
        #Incrimento de J para controle de nivel.
        j = j+1
        #Incremento de i para varrer os pontos.
        i = i+1
	    #Fim do loop zero os pontos.
        if (i ==len(data.points)):
            if (yellow == 1 and red == 0):
                colisao = yellow
            if (yellow == 1 and red == 2):
                colisao = red
            if (yellow == 0 and red == 0):
                colisao = green            
            i = 0
            nivel = 0
            vetx=[]
            vety=[]
            vetz=[]
    colisao_visual.points = list(aux_colisao_visual)

def getdata():
	global colisao,old_colisao,proximity
	rospy.init_node("detect_colision",anonymous=True)
    #Altera aqui para o topico da nuvem de pontos
	rospy.Subscriber("/velodyne_points",PointCloud,callback)
	#Publica no topico colisao true se detectar colisao e false se nao.
	pub = rospy.Publisher("colisao",Int16,queue_size = 10)
	#Publica em qual setor do robo ocorreu a colisao.
	col = rospy.Publisher("pontos",Int32,queue_size = 10)
	#Topico que publica os pontos de colisao
	visual = rospy.Publisher("Pontos_colisao",PointCloud,queue_size =10)
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
            #so altera valor no topico colisao caso ele mude
	        if (old_colisao != colisao):
        	    old_colisao = colisao
	            pub.publish(old_colisao)
		    col.publish(pontos_colisao)
		visual.publish(colisao_visual)
	   	rate.sleep()
if __name__== '__main__':
	try:
		getdata()
	except rospy.ROSInterruptException:
		pass
