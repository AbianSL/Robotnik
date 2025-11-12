#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Robótica Computacional - 
# Grado en Ingeniería Informática (Cuarto)
# Práctica: Resolución de la cinemática inversa mediante CCD
#           (Cyclic Coordinate Descent).

import sys
from math import *
import numpy as np
import matplotlib.pyplot as plt
import colorsys as cs

# ******************************************************************************
# Declaración de funciones

def muestra_origenes(O,final=0):
  # Muestra los orígenes de coordenadas para cada articulación
  print('Origenes de coordenadas:')
  for i in range(len(O)):
    print('(O'+str(i)+')0\t= '+str([round(j,3) for j in O[i]]))
  if final:
    print('E.Final = '+str([round(j,3) for j in final]))

def muestra_robot(O,obj):
  # Muestra el robot graficamente
  plt.figure()
  plt.xlim(-L,L)
  plt.ylim(-L,L)
  T = [np.array(o).T.tolist() for o in O]
  for i in range(len(T)):
    plt.plot(T[i][0], T[i][1], '-o', color=cs.hsv_to_rgb(i/float(len(T)),1,1))
  plt.plot(obj[0], obj[1], '*')
  plt.pause(0.0001)
  plt.show()
  
#  input()
  plt.close()

def matriz_T(d,th,a,al):
  return [[cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th)]
         ,[sin(th),  cos(th)*cos(al), -sin(al)*cos(th), a*sin(th)]
         ,[      0,          sin(al),          cos(al),         d]
         ,[      0,                0,                0,         1]
         ]

def cin_dir(th,a):
  #Sea 'th' el vector de thetas
  #Sea 'a'  el vector de longitudes
  T = np.identity(4)
  o = [[0,0]]
  for i in range(len(th)):
    T = np.dot(T,matriz_T(0,th[i],a[i],0))
    tmp=np.dot(T,[0,0,0,1])
    o.append([tmp[0],tmp[1]])
  return o

def normalize_angle(angle):
    while abs(angle) > np.pi:
        if angle > 0:
            angle -= 2*np.pi
        else:
            angle += 2*np.pi
    return angle

def calc_rotation(objetivo, last, length, it):
    new_values_objective = [objetivo[0] - last[length - it - 1][0], 
                            objetivo[1] - last[length - it - 1][1]]
    new_values_O = [last[length][0] - last[length - it - 1][0] , 
                    last[length][1] - last[length - it - 1][1] ]
    alpha1 = np.atan2(new_values_O[1], new_values_O[0])
    alpha2 = np.atan2(new_values_objective[1], new_values_objective[0])
    new_alpha = alpha2 - alpha1
    return new_alpha

def calc_distance(objetivo, last, length, it) -> float:
    new_values_objective = [objetivo[0] - last[length - it - 1][0], 
                            objetivo[1] - last[length - it - 1][1]]
    distance_r = [new_values_objective[0] - last[length][0] - last[length - it - 1][0], 
                    new_values_objective[1] - last[length][1] - last[length - it - 1][1] ]
    all_angle = 0
    for i in range(length):
        all_angle += th[i]
    array = [np.cos(all_angle), np.sin(all_angle)]
    result = np.dot(array, distance_r)
    print(f"Distance to move prismatic joint {length - it}: {result}")
    return result

# ******************************************************************************
# Cálculo de la cinemática inversa de forma iterativa por el método CCD

# valores articulares arbitrarios para la cinemática directa inicial
th=[0.,0.,0.]
a =[5.,5.,5.]
limits = [[-np.pi / 2, np.pi / 2],
          [1, 4],
          [-np.pi / 8, np.pi / 8]]
spot_type = [False, True, False] # False = rotational, True = prismatic
L = sum(a) # variable para representación gráfica
EPSILON = .01

#plt.ion() # modo interactivo

# introducción del punto para la cinemática inversa
if len(sys.argv) != 3:
  sys.exit("python " + sys.argv[0] + " x y")
objetivo=[float(i) for i in sys.argv[1:]]
O=cin_dir(th,a)
#O=zeros(len(th)+1) # Reservamos estructura en memoria
 # Calculamos la posicion inicial
print ("- Posicion inicial:")
muestra_origenes(O)

dist = float("inf")
prev = 0.
iteracion = 1
while (dist > EPSILON and abs(prev-dist) > EPSILON/100.):
  prev = dist
  O=[cin_dir(th,a)]
  # Para cada combinación de articulaciones:
  for i in range(len(th)):
    # cálculo de la cinemática inversa:
    if spot_type[len(th) - 1 - i]:
        distance = calc_distance(objetivo, O[-1], len(th), i)
        a[len(th) - 1 - i] += distance
        if (a[len(th) - 1 - i] < limits[len(th) - 1 - i][0]):
            a[len(th) - 1 - i] = limits[len(th) - 1 - i][0]
        if (a[len(th) - 1 - i] > limits[len(th) - 1 - i][1]):
            a[len(th) - 1 - i] = limits[len(th) - 1 - i][1]
    else:
        alpha = calc_rotation(objetivo, O[-1], len(th), i)
        th[len(th) - 1 - i] += alpha
        th[len(th) - 1 - i] = normalize_angle(th[len(th) - 1 - i])
    O.append(cin_dir(th,a))

  dist = np.linalg.norm(np.subtract(objetivo,O[-1][-1]))
  print ("\n- Iteracion " + str(iteracion) + ':')
  muestra_origenes(O[-1])
  muestra_robot(O,objetivo)
  print ("Distancia al objetivo = " + str(round(dist,5)))
  iteracion+=1
  O[0]=O[-1]

if dist <= EPSILON:
  print ("\n" + str(iteracion) + " iteraciones para converger.")
else:
  print ("\nNo hay convergencia tras " + str(iteracion) + " iteraciones.")
print ("- Umbral de convergencia epsilon: " + str(EPSILON))
print ("- Distancia al objetivo:          " + str(round(dist,5)))
print ("- Valores finales de las articulaciones:")
for i in range(len(th)):
  print ("  theta" + str(i+1) + " = " + str(round(th[i],3)))
for i in range(len(th)):
  print ("  L" + str(i+1) + "     = " + str(round(a[i],3)))
