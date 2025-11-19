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
    alpha1 = atan2(new_values_O[1], new_values_O[0])
    alpha2 = atan2(new_values_objective[1], new_values_objective[0])
    new_alpha = alpha2 - alpha1
    return new_alpha

def calc_distance(objetivo, last, length, it) -> float:
    distance_r = [objetivo[0] - last[-1][0], 
                  objetivo[1] - last[-1][1]]
    all_angle = 0
    for i in range(length - it):
        all_angle += th[i]
    array = [np.cos(all_angle), np.sin(all_angle)]
    result = np.dot(array, distance_r)
    return result

def rad_to_grade(radians):
    grade = radians * (180 / np.pi)
    return grade

def grade_to_rad(grades):
    radians = grades * (np.pi / 180)
    return radians

# ******************************************************************************
# Cálculo de la cinemática inversa de forma iterativa por el método CCD

grade90 = grade_to_rad(90)
grade25 = grade_to_rad(25)
grade65 = grade_to_rad(65)
grade140 = grade_to_rad(140)

# valores articulares arbitrarios para la cinemática directa inicial
option_th = []
option_a = []
option_limits = []
option_spot_type = []

# coordenadas objetivo: -2.3 7
option_th.append([grade_to_rad(15), 0., grade_to_rad(25), 0.])
option_a.append([2., 1., 3., 1.] )
option_limits.append([[-grade90, grade90],
          [0, 3],
          [-grade25, grade65],
          [-grade90, grade90]])
option_spot_type.append([False, True, False, False]) # False = rotational, True = prismatic

# coordenadas objetivo: 3 5
option_th.append([grade_to_rad(10), grade_to_rad(25), 0, 0.])
option_a.append([2., 3., 1., 1.]) 
option_limits.append([[-grade90, grade90],
          [-grade25, grade65],
          [0, 3],
          [-grade90, grade90]])
option_spot_type.append([False, False, True, False])

# coordenadas objetivo: -6 5
option_th.append([grade_to_rad(5), grade_to_rad(45), grade_to_rad(25), 0.])
option_a.append([3., 1., 3., 1.]) 
option_limits.append([[-grade140, grade140],
          [0, 3],
          [-grade90, grade90],
          [0, 5]])
option_spot_type.append([False, True, False, True])

# configuracion de prueba
option_th.append([0., 0., 0., 0.]) 
option_a.append([5., 5., 5., 5.])
option_limits.append([[-grade90, grade90],
          [0, 5],
          [-grade90, grade90],
          [0, 5]])
option_spot_type.append([False, True, False, True]) 

# 0 -> primera opción, 3 -> configuracion de prueba
option_selected = 0
th= option_th[option_selected]
a = option_a[option_selected]
limits = option_limits[option_selected]
spot_type = option_spot_type[option_selected] 
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
        print("\nPrismatic joint adjustment:")
        print(f"a[{len(th) - 1 - i}] = {a[len(th) - 1 - i]}")
        print(f"th[{len(th) - 1 - i}] = {th[len(th) - 1 - i]}")
    else:
        alpha = calc_rotation(objetivo, O[-1], len(th), i)
        th[len(th) - 1 - i] += alpha
        th[len(th) - 1 - i] = normalize_angle(th[len(th) - 1 - i])
        if (th[len(th) - 1 - i] < limits[len(th) - 1 - i][0]):
            th[len(th) - 1 - i] = limits[len(th) - 1 - i][0]
        if (th[len(th) - 1 - i] > limits[len(th) - 1 - i][1]):
            th[len(th) - 1 - i] = limits[len(th) - 1 - i][1]
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
  print ("  theta" + str(i+1) + " = " + str(round(rad_to_grade(th[i]),3)))
for i in range(len(th)):
  print ("  L" + str(i+1) + "     = " + str(round(a[i],3)))
