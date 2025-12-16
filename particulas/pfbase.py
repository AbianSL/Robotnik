#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Robótica Computacional -
# Grado en Ingeniería Informática (Cuarto)
# Práctica: Filtros de particulas.

from math import *
from robot import *
import random
import numpy as np
import matplotlib.pyplot as plt
import sys
import select
from datetime import datetime
from config import *
# ******************************************************************************
# Declaración de funciones

def distancia(a,b):
  # Distancia entre dos puntos (admite poses)
  return np.linalg.norm(np.subtract(a[:2],b[:2]))

def angulo_rel(pose,p):
  # Diferencia angular entre una pose y un punto objetivo 'p'
  w = atan2(p[1]-pose[1],p[0]-pose[0])-pose[2]
  while w >  pi: w -= 2*pi
  while w < -pi: w += 2*pi
  return w

def pinta(secuencia,args):
  # Dibujar una secuencia de puntos
  t = np.array(secuencia).T.tolist()
  plt.plot(t[0],t[1],args)

def mostrar(objetivos,trayectoria,trayectreal,filtro):
  # Mostrar mapa y trayectoria
  plt.ion() # modo interactivo
  plt.clf()
  plt.axis('equal')
  # Fijar los bordes del gráfico
  objT   = np.array(objetivos).T.tolist()
  bordes = [min(objT[0]),max(objT[0]),min(objT[1]),max(objT[1])]
  centro = [(bordes[0]+bordes[1])/2.,(bordes[2]+bordes[3])/2.]
  radio  = max(bordes[1]-bordes[0],bordes[3]-bordes[2])
  plt.xlim(centro[0]-radio,centro[0]+radio)
  plt.ylim(centro[1]-radio,centro[1]+radio)
  # Representar mapa
  for p in filtro:
    dx = cos(p.orientation)*.05
    dy = sin(p.orientation)*.05
    plt.arrow(p.x,p.y,dx,dy,head_width=.05,head_length=.05,color='k')
  pinta(trayectoria,'--g')
  pinta(trayectreal,'-r')
  pinta(objetivos,'-.ob')
  p = hipotesis(filtro)
  dx = cos(p[2])*.05
  dy = sin(p[2])*.05
  plt.arrow(p[0],p[1],dx,dy,head_width=.075,head_length=.075,color='m')
  # Mostrar y comprobar pulsaciones de teclado:
  # plt.show()
  # if sys.stdin in select.select([sys.stdin],[],[],.01)[0]:
  #   line = sys.stdin.readline()
  # input()
  # plt.close()
  plt.draw()
  plt.pause(0.001)

def genera_filtro(num_particulas, balizas, real, centro=CENTER, radio=RADIUS):
  # Inicialización de un filtro de tamaño 'num_particulas', cuyas partículas
  # imitan a la muestra dada y se distribuyen aleatoriamente sobre un área dada.
  filter = []
  for i in range(num_particulas):
    p = real.copy()
    angle = random.uniform(0,2*pi)
    r = radio*sqrt(random.uniform(0,1))
    p.set(centro[0]+r*cos(angle),
          centro[1]+r*sin(angle),
          random.uniform(0,2*pi))
    filter.append(p)
  return filter

def dispersion(filtro):
  # Dispersion espacial del filtro de particulas
  xs = [p.x for p in filtro]
  ys = [p.y for p in filtro]
  return np.std(xs) + np.std(ys) 

def peso_medio(filtro):
  # Peso medio normalizado del filtro de particulas
  return sum([p.weight for p in filtro])/len(filtro)

# ******************************************************************************

random.seed(0)
# Definición de trayectorias:
trayectorias = [
    [[0,2],[4,2]],
    [[2,4],[4,0],[0,0]],
    [[2,4],[2,0],[0,2],[4,2]],
    [[2+2*sin(.4*pi*i),2+2*cos(.4*pi*i)] for i in range(5)],
    [[2+2*sin(.8*pi*i),2+2*cos(.8*pi*i)] for i in range(5)],
    [[2+2*sin(1.2*pi*i),2+2*cos(1.2*pi*i)] for i in range(5)],
    [[2*(i+1),4*(1+cos(pi*i))] for i in range(6)],
    [[2+.2*(22-i)*sin(.1*pi*i),2+.2*(22-i)*cos(.1*pi*i)] for i in range(20)],
    [[2+(22-i)/5*sin(.1*pi*i),2+(22-i)/5*cos(.1*pi*i)] for i in range(20)]
    ]

# Definición de los puntos objetivo:
if len(sys.argv)<2 or int(sys.argv[1])<0 or int(sys.argv[1])>=len(trayectorias):
  sys.exit(sys.argv[0]+" <indice entre 0 y "+str(len(trayectorias)-1)+">")
objetivos = trayectorias[int(sys.argv[1])]

# Definición de constantes:
real = robot()
real.set_noise(NOISE[0], NOISE[1], NOISE[2]) # Ruido lineal / radial / de sensado
real.set(*P_INICIAL)

#inicialización del filtro de partículas y de la trayectoria
balizas = objetivos
filter = genera_filtro(N_INICIAL, balizas, real)
pose = hipotesis(filter)
trayectoria = [pose]

trayectreal = [real.pose()]

tiempo  = 0.
espacio = 0.
for punto in objetivos:
  giro_acumulado = 0.0
  while distancia(trayectoria[-1],punto) > EPSILON and len(trayectoria) <= 1000:

    w = angulo_rel(pose,punto)
    if w > W:  w =  W
    if w < -W: w = -W
    v = distancia(pose,punto)
    if (v > V): v = V
    if (v < 0): v = 0
    if HOLONOMICO:
      if GIROPARADO and abs(w) > .01:v = 0
      real.move(w,v)
    else:
      real.move_triciclo(w,v,LONGITUD)

    giro_acumulado += w
    print(f"Giro acumulado: {giro_acumulado}") 
    if abs(giro_acumulado) > UMBRAL_BUCLE:
        print("Bucle detectado: ignorando punto objetivo, pasando al siguiente")
        break;

    #seleccionar pose
    for p in filter:
      if HOLONOMICO:
        p.move(w, v)
      else:
        p.move_triciclo(w, v, LONGITUD)

    # Seleccionar hipótesis de localización y actualizar la trayectoria
    measurements = real.sense(balizas)
    for p in filter:
      p.measurement_prob(measurements, balizas)
    
    calidad_actual = peso_medio(filter)
    print(f"calidad {calidad_actual}\n\n")
    if calidad_actual < UMBRAL_CALIDAD:
        print("ROBOT PERDIDO. Calidad insuficiente (" + str(round(calidad_actual, 4)) + ").")
        print(" Pasando al siguiente objetivo")
        break
    
    # remuestreo
    d = dispersion(filter)
    n_dinamico = int(N_MIN + d * FACTOR_DIN)
    if n_dinamico > N_MAX: n_dinamico = N_MAX
    if n_dinamico < N_MIN: n_dinamico = N_MIN
    filter = resample(filter, n_dinamico)
    
    pose = hipotesis(filter)
    trayectoria.append(pose)

    trayectreal.append(real.pose())
    mostrar(objetivos, trayectoria, trayectreal, filter)

    espacio += v
    tiempo  += 1

if len(trayectoria) > 1000:
  print ("<< ! >> Puede que no se haya alcanzado la posicion final.")
print ("Recorrido: "+str(round(espacio,3))+"m / "+str(tiempo/FPS)+"s" )
print ("Error medio de la trayectoria: "+str(round(sum(\
    [distancia(trayectoria[i],trayectreal[i])\
    for i in range(len(trayectoria))])/tiempo,3))+"m" )
input()

