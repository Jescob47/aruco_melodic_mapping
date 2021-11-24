#!/usr/bin/python

import rospy
import numpy as np
import tf 
from math import sqrt
from aruco_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

false_id_marker =[177] #eliminar los id que se sabe que no estan en el mapa
markers_detected = [] #esta lista va a contener la informacion de los marcadores que se han encontrado
aver_markers_pos = {} #recolecta todos los datos del determinado id para promediarlos
static = True

dic_prop = [("id",-1), 
("posx",0.0), 
("posy",0.0), 
("posz",0.0), 
("quatx",0.0), 
("quatx",0.0), 
("quaty",0.0), 
("quatz",0.0), 
("quatw",0.0), 
("visible", False),
("parent_frame", " "),
("child_frame", " "),
("count", 0),
("flag", False)
]#contiene las propiedades de cada amrcador que luego se ponen en markers

camera_position = [{
"id":-1,
"posx":0.0, 
"posy":0.0, 
"posz":0.0, 
"quatx":0.0, 
"quatx":0.0, 
"quaty":0.0, 
"quatz":0.0, 
"quatw":0.0,
"parent_frame":"/map ",
"child_frame": "/camera",
"count":1
}]#tiene las propiedades de la camara

marker_counter = 0#cuantos marcadores ha detectado
marker_size = 0.085 #tamano en metros del marcador
first_marker_detected = False #si no a dectectado el primer marcador es falso
pos_last_calc = 0
last_detected =[]
global_cont = 0
pos_tol = 0.002
eu_tol = 0.02

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

def pos_calc(child, c, parent,p): #c es el index de child y p es el index de parent
#pasar del sistema coordenado child a parent. La info que debe tener el child debe contener la info del marker_array desde afuera
	
	if child[c]["child_frame"] == "/camera":#cuando se quiere encontrar la ubicacion de la camara
		flagchild = 1
		flagparent = 0

	else: #cuando se quiere encontrar la posicion de un aruco con respecto al global
		
		flagchild = 0
		flagparent = 0
	
	matrizchild = matriz_tf(child,c,flagchild)
	matrizparent = matriz_tf(parent,p,flagparent)
	matriz = np.dot(matrizparent, matrizchild)
	transl, quat = quat_trans_from_matriz(matriz)
	child[c]["posx"] = transl[0]
	child[c]["posy"] = transl[1]
	child[c]["posz"] = transl[2]
	child[c]["quatx"] = quat[0]
	child[c]["quaty"] = quat[1]
	child[c]["quatz"] = quat[2]
	child[c]["quatw"] = quat[3]

	return child

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

def sum_vect(vect, i, aruco1, index): #funcion para encontrar que aruco esta mas cerca de la camara y usa la info del array como vect
#devuelve la posicion del aruco mas cercano a la camra en marker array
	if aruco1 == 0:

		aruco1 = sqrt((vect[i].pose.pose.position.x**2)+(vect[i].pose.pose.position.y**2)+(vect[i].pose.pose.position.z**2))
		index = i
		return index,aruco1

	else:

		aruco2 = sqrt((vect[i].pose.pose.position.x**2)+(vect[i].pose.pose.position.y**2)+(vect[i].pose.pose.position.z**2))

		if aruco2 < aruco1:

			aruco1 = aruco2
			index = i
			return index, aruco1
		
		else:
			return index,aruco1

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

def matriz_tf(data, i, flag): #saca la matriz de tf normal (de marcador a camara) o inversa (de camara a marcador)
	#si flag es 0 es porque se necesita la matriz normal, si es 1 es que se necesita la invertida

	tl = tf.TransformerROS()
	px=data[i]["posx"]
	py=data[i]["posy"]
	pz=data[i]["posz"]
	rx=data[i]["quatx"]
	ry=data[i]["quaty"]
	rz=data[i]["quatz"]
	w=data[i]["quatw"]

	matriz=np.array(tl.fromTranslationRotation((px,py,pz), (rx,ry,rz,w))) #esta es la matriz del sistema global a los arucos

	if flag:

		matrizrotinver = np.linalg.inv(matriz[0:3,0:3])#hago la transpuesta o inversa de la matriz rotacional
		transinver = np.dot(matrizrotinver,-matriz[0:3,3]) #vuelvo negativo el vector translacion y lo multiplico por la matriz rotacional inversa
		matrizinver = np.insert(matrizrotinver,matrizrotinver.shape[1],transinver,1)#agrego a la matriz de rotacion inversa el vector translacion y queda matriz 3x4
		matrizinver = np.insert(matrizinver,matrizinver.shape[0],np.array([0,0,0,1]),0)#agrego al final de las filas de la matriz anterior el 0,0,0,1 para que quede la matriz 4x4

		return matrizinver
	else:
		return matriz

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

def quat_trans_from_matriz(matriz): #devuelve los valores de translacion y orientacion de una matriz
		quat = tf.transformations.quaternion_from_matrix(matriz)
		translation = tf.transformations.translation_from_matrix(matriz)

		return translation, quat

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

def del_false_id(array): #elimina los marcadores con id que estan en la lista de false id marker

	global false_id_marker

	for i in array:
		if false_id_marker.count(i.id) > 0:
			array.remove(i)
	
	return array
		
#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

def tolerance(array, index, div):

	global pos_tol, eu_tol

	dic_results = {num: {} for num in range(len(array[index]))}
	results = {num: 0 for num in range(div)}


	for i in range(len(array[index])):

		for y in range(len(array[index][i])-1):

			x = y + 1

			while x < len(array[index][i]):

				r = abs(array[index][i][y] - array[index][i][x])

				if i <= 3: #para las tolerancias de las posiciones

					if r <= pos_tol:

						results[x] = results[x] + 1
						results[y] = results[y] + 1
						

				else: #para las tolerancias de los euler

					if r <= eu_tol:

						results[x] = results[x] + 1
						results[y] = results[y] + 1
	
				x += 1
		
		dic_results[i] = results
		results = {num: 0 for num in range(div)}

	return dic_results

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

def quat_euler(data, index,flag):#saca euler de cuaternio o quaternio de euler segun la bandera
	
	if flag == False: #saca euler del quaternio

		qx = data[index]["quatx"]
		qy = data[index]["quaty"]
		qz = data[index]["quatz"]
		qw = data[index]["quatw"]

		euler = tf.transformations.euler_from_quaternion([qx,qy,qz,qw])

		return euler

	else: #sino, saca el quaternio de euler

		roll = (sum(data[index][3]))/len(data[index][3])
		pitch = (sum(data[index][4]))/len(data[index][4])
		yaw = (sum(data[index][5]))/len(data[index][5])

		quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

		return quat

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

def pass_info(new_data, old_data, inew, iold): #pasa la informacion del tipo marker.msg a la usada en el codigo o de old data a new dat
	if new_data[inew]["child_frame"] != "/camera":
		new_data[inew]["id"] = old_data[iold].id

	new_data[inew]["posx"] = old_data[iold].pose.pose.position.x
	new_data[inew]["posy"] = old_data[iold].pose.pose.position.y
	new_data[inew]["posz"] = old_data[iold].pose.pose.position.z
	new_data[inew]["quatx"] = old_data[iold].pose.pose.orientation.x
	new_data[inew]["quaty"] = old_data[iold].pose.pose.orientation.y
	new_data[inew]["quatz"] = old_data[iold].pose.pose.orientation.z
	new_data[inew]["quatw"] = old_data[iold].pose.pose.orientation.w

	return new_data

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

def sum_rest(a, b): #se usa para la suma vectorial entre 2 marcadores, para adecuar el signo de las componentes x y z

	if (a > 0 and b > 0) or (a < 0 and b < 0):
		result = a - b
	else:
		result = a + b
	
	return result

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

def average(av_data, av_id, data, data_id, div, delete):#calcula el promedio de las posiciones, guarda en cada frame que aparece el marcador la posicion y luego la promedia y permite publicarlo
				
	av_data[av_id][0].append(data[data_id]["posx"])
	av_data[av_id][1].append(data[data_id]["posy"])
	av_data[av_id][2].append(data[data_id]["posz"])

	euler = quat_euler(data,data_id,False)

	av_data[av_id][3].append(euler[0])
	av_data[av_id][4].append(euler[1])
	av_data[av_id][5].append(euler[2])

	if data[data_id]["count"] == div: #cuando se llega al numero solicitado de ubicaciones, se procede a cambiar el valor de data

		result = tolerance(av_data, av_id, div)
		cont = 0 #este contador indica si es 0 es que esta en el diccionario de las x, si es 1 es que esta en y etc
		flag = True
		c = 0

		for i in result.values():

			if sum(i.values()) == 0: #si la suma es 0 es porque ninguno tuvo la tolerancia con otro valor

				delete.append(data_id)
				flag = False

				break

			else:

				val = list(i.values())
				key = list(i.keys())
				key = key[::-1]

				for y in key:

					if val[y] == 0:

						del av_data[av_id][cont][y]
			cont += 1

		if flag:

			data[data_id]["posx"] = (sum(av_data[av_id][0]))/len(av_data[av_id][0]) #se suma toda la lista de x y se divide el resultado para sacar el promedio
			data[data_id]["posy"] = (sum(av_data[av_id][1]))/len(av_data[av_id][1])
			data[data_id]["posz"] = (sum(av_data[av_id][2]))/len(av_data[av_id][2])

			quat = quat_euler(av_data,av_id,True) #se saca los quaternios de los valores de euler

			data[data_id]["quatx"] = quat[0] #se asigna los valores de euler a data
			data[data_id]["quaty"] = quat[1]
			data[data_id]["quatz"] = quat[2]
			data[data_id]["quatw"] = quat[3]

			for i in range(len(av_data[av_id])):

				av_data[av_id][i][0] = (sum(av_data[av_id][i]))/len(av_data[av_id][i]) #se ubica en la posicion 0 de la lista x,y,z,ex,ey,ez el valor promediado

				del av_data[av_id][i][1::] #se eliminan los otros valores desde la posicion 1 hasta la ultima

			if data[data_id]["id"] != -1:

				data[data_id]["visible"] = True
				data[data_id]["flag"] = True
			
	return data, av_data, delete

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

def imageP(array): #se encarga de agregar todos los marcadores nuevos y la camara con su ubicacion respecto al mundo

	global markers_detected, dic_prop, first_marker_detected, camera_position, marker_counter, pos_last_calc, last_detected, aver_markers_pos, global_cont, static
	
	marker_array = array.markers 
	pos_rep_mark = [] #guarda en una lista las posiciones de los marcadores en marker array que ya se habian encontrado
	visible_markers_before = {} #guarda en una lista la posicion i de markers detected del cual en la imagen anterior era visible y ahora es visible
	visible_markers_now = [] #guarda en una lista la posicion i de markers detected del cual en la imagen anterior era invisible y ahora visible
	visible_in_array =[] #posicion en marker array del aruco que era visible y sigue siendo visible, para luego sacar de este la pose de la camara
	to_calculate =[]
	near_aruco = 0 #valor de magnitud del vector posicion del aruco mas cercano a la camara
	index = "N/A"
	distance = 1.5
	times_calc = 4
	to_del =[]
	global_cont += 1

	print(static)

	marker_array = del_false_id(marker_array)

	if first_marker_detected == False and len(marker_array) > 0 and sqrt((marker_array[0].pose.pose.position.x**2)+(marker_array[0].pose.pose.position.y**2)+(marker_array[0].pose.pose.position.z**2))< distance: #la bandera es para saber si ya se habia detectado el 1er marcador
		#pongo la info del marcador global
		markers_detected.append(dict(dic_prop))
		markers_detected[0]["id"] = marker_array[0].id #pongo el id del primer marcador encontrado
		markers_detected[0]["quatw"] = 1.0 # ya toda la info esta en 0 para dar la ubicacion al primer marcador detectado como la global
		markers_detected[0]["visible"] = True
		markers_detected[0]["parent_frame"] =  "/map"
		markers_detected[0]["child_frame"] = "marker_%d" %markers_detected[0]["id"]
		markers_detected[0]["count"] = times_calc
		markers_detected[0]["flag"] = True
		first_marker_detected = True # como ya se detecto el primer marcador lo pongo en true para que no vuelva a entrar
		marker_counter += 1

	if pos_last_calc != "N/A":#se crea una lista independiente con los valores anteriores de ser el caso en que el proximo frame no contenga marcadores

		last_detected = [dict(markers_detected[i]) for i in range(len(markers_detected))] #se necesita solo cuando no encuentra marcadores y luego encuentra marcadores no visibles-visibles

	#de los marcadores actualmente visible, detecta cuales ya se habian visto !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!111
			
	for i in range(len(markers_detected)):

		visible = False

		for cont in range(len(marker_array)):

			if markers_detected[i]["id"] == marker_array[cont].id and sqrt((marker_array[cont].pose.pose.position.x**2)+(marker_array[cont].pose.pose.position.y**2)+(marker_array[cont].pose.pose.position.z**2))< distance: #encontro un marcador que ya se habia visto

				pos_rep_mark.append(cont)#pos rep mark tiene la posicion del id repetido en marker array
				visible = True

				if markers_detected[i]["visible"] == False:	#si el marcador tenia la visibilidad en falso, ahora que esta visible lo pone en verdadero
					#el primer aruco que se detecto siempre va a estar en global por eso se pregunta si el id es diferente del primero para que no cambie los valores
					
					if pos_last_calc != "N/A":

						if markers_detected[i]["id"]!= markers_detected[0]["id"]: #siempre y cuando no sea el primer marcador encontrado, se le cambia los valores
				
							markers_detected = pass_info(markers_detected,marker_array,i,cont)
							visible_markers_now.append(i) #guarda la posicion del marcador en markers detected que antes no estaba visible y ahora si
							markers_detected[i]["count"] += 1
							to_calculate.append(i)

						else:
							
							visible_markers_before[cont]=i #guarda la posicion del marcador en markers detected que antes era visible y ahora tambien y la posicion del mismo en marker array
							visible_in_array.append(cont)
							markers_detected[0]["visible"] = True
							markers_detected[0]["count"] = times_calc


					else:#esto pasa cuando no encontro marcadores, luego encontro 1 marcador que es nuevo y luego encontro el nuevo con uno viejo
						
						markers_detected[i]["visible"] = True
						markers_detected[i]["count"] = times_calc
						visible_markers_before[cont]=i #guarda la posicion del marcador en markers detected que antes era visible y ahora tambien y la posicion del mismo en marker array
						visible_in_array.append(cont) #guarda la posicion en maker array del aruco que antes era visible y ahora tambien			 

				else:

					visible_markers_before[cont]=i #guarda la posicion del marcador en markers detected que antes era visible y ahora tambien y la posicion del mismo en marker array
					visible_in_array.append(cont) #guarda la posicion en maker array del aruco que antes era visible y ahora tambien

				break
		
		if visible == False: #si no encontro el id en lo que esta viendo la camara pone la visibilidad del aruco en falso

			if markers_detected[i]["flag"] == False:#para eliminar los que no se han visto numero v de veces seguidas

				to_del.append(i)

			elif static == False:

				markers_detected[i]["count"] = 1
				markers_detected[i]["visible"] = False

				if markers_detected[i]["id"] != markers_detected[0]["id"]:

					for x in range(len(aver_markers_pos[markers_detected[i]["id"]])):

						del aver_markers_pos[markers_detected[i]["id"]][x][1::] #se eliminan los otros valores desde la posicion 1 hasta la ultima

	#agrega los marcadores nuevos a la lista!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	if len(pos_rep_mark) != 0:

		for i in range(len(marker_array)): #ya conocidas las posiciones de los marcadores repetidos se procede a poner los nuevos en markers detected
			
			if pos_rep_mark.count(i) == 0 and len(visible_in_array)!=0 and sqrt((marker_array[i].pose.pose.position.x**2)+(marker_array[i].pose.pose.position.y**2)+(marker_array[i].pose.pose.position.z**2))< distance: #si la posicion i esta en la lista es porque el id ya esta repetido y se continua con el siguiente

				markers_detected.append(dict(dic_prop)) #se agrega al final de la lista la posicion del nuevo marcador con respecto a la camara
				markers_detected = pass_info(markers_detected,marker_array,-1, i)
				markers_detected[-1]["child_frame"] = "marker_%d" %markers_detected[-1]["id"]
				markers_detected[-1]["count"] = 1
				markers_detected[-1]["visible"] = False
				marker_counter += 1
				to_calculate.append(marker_counter-1)
				aver_markers_pos[markers_detected[-1]["id"]] = [[],[],[],[],[],[]] #la pos 0 es x, la 1 es y, la 2 es z, la 3 es roll, la 4 es pitch, la 5 es yaw

			if visible_in_array.count(i) > 0: #esta parte encuentra cual de los arucos esta mas cerca a la camara que antes eran visibles y todavia estan visibles 

				index, near_aruco = sum_vect(marker_array, i, near_aruco, index) 
	
	pos_last_calc = index #si ocurre que en un frame no vio ningun marcador y en el siguiente encontro marcador, este seria la posicion del ultimo marcador que fue visible visible

	#ahora hay que calcular la posicion de los marcadores que antes no eran visibles y ahora si, como tambien la de la camara !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1

	if index != "N/A": 

		camera_position = pass_info(camera_position, marker_array, 0, index) # se le pone la informacion del aruco con respecto a la camara mas cercano que antes era visible y ahora tambien 		
		camera_position = pos_calc(camera_position, 0, markers_detected, visible_markers_before[index]) # se calcula la posicion de la camara con respecto al aruco visible que antes era visible
		camera_position[0]["count"] += 1

		camera_position[0]["count"] = 1
		camera_position[0]["parent_frame"] = "/map"

		publishMarker(camera_position, 0)
		publish_tf(camera_position, 0)

		for i in to_calculate: #ahora calculamos la posicion de los marcadores ahora visibles y antes no, con respecto a la posicion de la camara en el sistema global

			markers_detected[i]["parent_frame"] = "/map"
			markers_detected = pos_calc(markers_detected, i, camera_position, 0)
			markers_detected, aver_markers_pos, to_del = average(aver_markers_pos, markers_detected[i]["id"], markers_detected, i, times_calc, to_del)


		markers_to_pub = list(visible_markers_before.values()) + to_calculate #solo se publican los visibles que ya hayan sido calculadas todas las poses

		for i in markers_to_pub: #solo se publican los visibles que ya hayan sido calculadas todas las poses

			if markers_detected[i]["visible"] == True:
				publishMarker(markers_detected, i)
				publish_tf(markers_detected, i)

	else:#si no contiene marcadores para calcular la posicion, se utiliza la posicion pasada como si fuera la actual

		markers_detected = [dict(last_detected[i]) for i in range(len(last_detected))]

		for i in visible_markers_now:

			markers_detected[i]["visible"] = True #le pone la visibilidad en true para que en el proximo frame lo coja como si fuera un marcador que antes era visible y ahora tambien

	to_del.sort()
	to_del = to_del[::-1]

	for i in to_del:

		del aver_markers_pos[markers_detected[i]["id"]]
		del markers_detected[i]
		del last_detected[i]
		marker_counter -= 1
	

	

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

def publishMarker(publisher, index):#publica los cuadrados que representan el marcador
	
	global marker_size
	
	pub=rospy.Publisher("/aruco/pub_marker", Marker, queue_size=100)#publica en el topic pub marker los valores para dibujar en rviz el marcador

	viz_marker = Marker()

	viz_marker.header.frame_id = "/map" 

	viz_marker.header.stamp = rospy.Time.now()
	viz_marker.ns = "basic_shapes"
	viz_marker.id = publisher[index]["id"]
	viz_marker.type = viz_marker.CUBE
	viz_marker.action = viz_marker.ADD

	viz_marker.pose.position.x = publisher[index]["posx"]
	viz_marker.pose.position.y = publisher[index]["posy"]
	viz_marker.pose.position.z = publisher[index]["posz"]
		
	viz_marker.pose.orientation.x = publisher[index]["quatx"]
	viz_marker.pose.orientation.y = publisher[index]["quaty"]
	viz_marker.pose.orientation.z = publisher[index]["quatz"]
	viz_marker.pose.orientation.w = publisher[index]["quatw"]
		
	viz_marker.scale.x = marker_size
	viz_marker.scale.y = marker_size
	viz_marker.scale.z = 0.01

	viz_marker.color.r = 1
	viz_marker.color.g = 1
	viz_marker.color.b = 1
	viz_marker.color.a = 1

	pub.publish(viz_marker) #se publica la informacion

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

def publish_tf(data, i): #publica las tf de cada marcador para rviz

	tl = tf.TransformBroadcaster()

	px = data[i]["posx"]
	py = data[i]["posy"]
	pz = data[i]["posz"]
	rx = data[i]["quatx"]
	ry = data[i]["quaty"]
	rz = data[i]["quatz"]
	w = data[i]["quatw"]
	parent = "/map"
	child = data[i]["child_frame"]
	time = rospy.Time.now()

	tl.sendTransform((px,py,pz), (rx,ry,rz,w), time, child, parent)#envia en forma de mensaje tf la posicion y orientacion del sistema coordenado ingresado

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

def listener(): #escucha lo que aruco manda
	global markers_detected, dic_prop, num_markers, cast

	rospy.init_node('a_mapping', anonymous=True) #se inicia el nodo con nombre prueba1

	rospy.Subscriber('/aruco/markers', MarkerArray, imageP) #el solo llama a imageP cuando aparece un marcador, de resto no hace nada

	rate = rospy.Rate(0.1) #10 veces por segundo
	while not rospy.is_shutdown():#mientras no presiona control c corre el programa
		rate.sleep()

#///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

if __name__ == '__main__':

	try:
		listener()

	except rospy.ROSInterruptException:
		rospy.loginfo("ERROR")