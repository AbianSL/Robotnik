#! /usr/bin/env python3

# Robótica Computacional
# Grado en Ingeniería Informática (Cuarto)
# Práctica 5:
#     Simulación de robots móviles holonómicos y no holonómicos.

#localizacion.py

import sys
from math import *
from matplotlib.patches import Ellipse
from robot import robot
import random
import numpy as np
import matplotlib.pyplot as plt
import time
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

def mostrar(objetivos,ideal,trayectoria):
  # Mostrar objetivos y trayectoria:
  plt.figure('Trayectoria')
  plt.clf()
  plt.ion() # modo interactivo: show no bloqueante
  # Fijar los bordes del gráfico
  objT   = np.array(objetivos).T.tolist()
  trayT  = np.array(trayectoria).T.tolist()
  ideT   = np.array(ideal).T.tolist()
  bordes = [min(trayT[0]+objT[0]+ideT[0]),max(trayT[0]+objT[0]+ideT[0]),
            min(trayT[1]+objT[1]+ideT[1]),max(trayT[1]+objT[1]+ideT[1])]
  centro = [(bordes[0]+bordes[1])/2.,(bordes[2]+bordes[3])/2.]
  radio  = max(bordes[1]-bordes[0],bordes[3]-bordes[2])*.75
  plt.xlim(centro[0]-radio,centro[0]+radio)
  plt.ylim(centro[1]-radio,centro[1]+radio)
  # Representar objetivos y trayectoria
  idealT = np.array(ideal).T.tolist()
  plt.plot(idealT[0],idealT[1],'-g', label='Ideal (EKF)')
  plt.plot(trayectoria[0][0],trayectoria[0][1],'or')
  
  # Dibujar trayectoria real
  plt.plot(trayT[0], trayT[1], '-r', alpha=0.5, label='Real')
  
  # Dibujar orientación solo del último punto para evitar saturación
  r = radio * .1
  p = trayectoria[-1]
  plt.plot([p[0],p[0]+r*cos(p[2])],[p[1],p[1]+r*sin(p[2])],'-r', linewidth=2)
  
  objT   = np.array(objetivos).T.tolist()
  plt.plot(objT[0],objT[1],'-.ob', label='Objetivos')
  plt.legend()
  plt.pause(0.001)
  plt.show()


def localizacion(balizas, real, ideal, P, Q, R_var, centro, radio, mostrar):
 
  # P -> Matriz de covarianza inicial
  # Q -> Ruido de proceso
  R_std = sqrt(R_var) 
  P = P + Q
  
  x_est = np.array(ideal.pose())
  z_real = np.array(real.senseDistance(balizas))

  for i, baliza in enumerate(balizas):
      bx, by = baliza
      
      dx = x_est[0] - bx
      dy = x_est[1] - by
      dist_pred = sqrt(dx**2 + dy**2)
      
      if dist_pred == 0: continue
      
      z = z_real[i]
      y = z - dist_pred
      
      # Jacobiana H
      H = np.array([dx/dist_pred, dy/dist_pred, 0.0])
     
      # @ -> Multiplicación de matrices
      S = H @ P @ H.T + R_var
      K = P @ H.T * (1.0 / S)
      
      x_est = x_est + K * y
      
      while x_est[2] >  pi: x_est[2] -= 2*pi
      while x_est[2] < -pi: x_est[2] += 2*pi
      
      I = np.eye(3)
      # np.outer -> Producto externo de vectores
      P = (I - np.outer(K, H)) @ P

  ideal.set(x_est[0], x_est[1], x_est[2])
  
  if mostrar:
    plt.figure('Localizacion')
    plt.clf()
    # Dibujar balizas
    balT = np.array(balizas).T.tolist()
    plt.plot(balT[0], balT[1], 'or', ms=10)
    # Dibujar robot Real e Ideal
    plt.plot(real.x, real.y, 'D', c='#00ff00', ms=10, mew=2, label='Real')
    plt.plot(ideal.x, ideal.y, 'D', c='#ff00ff', ms=10, mew=2, label='EKF')
    
    # Dibujar elipse de incertidumbre (Covarianza P)
    # Tomamos los eigenvalores de la submatriz 2x2 (x,y)
    cov_xy = P[0:2, 0:2]
    vals, vecs = np.linalg.eigh(cov_xy)
    angle = degrees(atan2(vecs[1, 0], vecs[0, 0]))
    width, height = 2 * np.sqrt(vals) # 2 desviaciones estándar (95%)
    
    ell = Ellipse(xy=(ideal.x, ideal.y), width=width, height=height, angle=angle, 
                  edgecolor='blue', fc='None', lw=2, label='Covarianza')
    plt.gca().add_patch(ell)
    
    # Ajustar vista
    plt.xlim(centro[0]-radio, centro[0]+radio)
    plt.ylim(centro[1]-radio, centro[1]+radio)
    plt.legend()
    plt.pause(0.001)
    plt.show()

def localizacion_ekf(balizas, real, ideal, P, Q, R_var, centro, radio, mostrar_graficos):
  # Predicción: P = P + Q (ya aplicado en ejecutar_simulacion antes de llamar)
  # Nota: P ya viene actualizado con Q desde fuera
  
  x_est = np.array(ideal.pose())
  z_real = np.array(real.senseDistance(balizas))

  for i, baliza in enumerate(balizas):
      bx, by = baliza
      dx = x_est[0] - bx
      dy = x_est[1] - by
      dist_pred = sqrt(dx**2 + dy**2)
      
      if dist_pred == 0: continue
      
      z = z_real[i]
      y = z - dist_pred  # Innovación
      
      H = np.array([dx/dist_pred, dy/dist_pred, 0.0])
      S = H @ P @ H.T + R_var
      K = P @ H.T * (1.0 / S)
      x_est = x_est + K * y
      
      while x_est[2] >  pi: x_est[2] -= 2*pi
      while x_est[2] < -pi: x_est[2] += 2*pi
      
      I = np.eye(3)
      P = (I - np.outer(K, H)) @ P

  ideal.set(x_est[0], x_est[1], x_est[2])
  
  if mostrar_graficos:
    plt.figure('Localizacion')
    plt.clf()
    balT = np.array(balizas).T.tolist()
    plt.plot(balT[0], balT[1], 'or', ms=10)
    plt.plot(real.x, real.y, 'D', c='#00ff00', ms=10, mew=2, label='Real')
    plt.plot(ideal.x, ideal.y, 'D', c='#ff00ff', ms=10, mew=2, label='EKF')
    
    cov_xy = P[0:2, 0:2]
    vals, vecs = np.linalg.eigh(cov_xy)
    angle = degrees(atan2(vecs[1, 0], vecs[0, 0]))
    width, height = 2 * np.sqrt(vals)
    
    ell = Ellipse(xy=(ideal.x, ideal.y), width=width, height=height, angle=angle, 
                  edgecolor='blue', fc='None', lw=2, label='Covarianza')
    plt.gca().add_patch(ell)
    
    plt.xlim(centro[0]-radio, centro[0]+radio)
    plt.ylim(centro[1]-radio, centro[1]+radio)
    plt.legend()
    plt.pause(0.001)
  
  return P


def ejecutar_simulacion(objetivos, pose_inicial, ruidos, mostrar_gui=True):
    """
    objetivos: lista de [x, y]
    pose_inicial: [x, y, theta]
    ruidos: diccionario {'lin': float, 'ang': float, 'sense': float}
    """
    
    # Configuración base
    V_LINEAL  = .7
    V_ANGULAR = 140.
    FPS       = 10.
    EPSILON   = .1
    V = V_LINEAL/FPS
    W = V_ANGULAR*pi/(180*FPS)
    
    # Inicialización Robots
    ideal = robot()
    ideal.set_noise(0,0,0)
    ideal.set(*pose_inicial)
    
    real = robot()
    # Configurar ruidos desde el CLI
    real.set_noise(ruidos['lin'], ruidos['ang'], ruidos['sense']) 
    real.set(*pose_inicial)
    
    tray_real = [real.pose()]
    tray_ideal = [ideal.pose()]
    
    # Inicialización Matrices EKF
    P = np.eye(3) * 0.1 # Covarianza inicial pequeña
    # Q: Ruido de proceso (movimiento)
    Q = np.diag([ruidos['lin']**2, ruidos['lin']**2, ruidos['ang']**2])
    # R_var: Varianza del ruido de medida
    R_var = ruidos['sense']**2

    tiempo = 0.
    espacio = 0.
    distanciaObjetivos = []
    
    random.seed(0)
    tic = time.time()
    
    for punto in objetivos:
        while distancia(tray_ideal[-1], punto) > EPSILON and len(tray_ideal) <= 1000:
            pose = ideal.pose()
            
            # Control de movimiento
            w = angulo_rel(pose, punto)
            if w > W:  w =  W
            if w < -W: w = -W
            v = distancia(pose, punto)
            if (v > V): v = V
            if (v < 0): v = 0
            
            # Movimiento (Holonómico simplificado)
            ideal.move(w, v)
            real.move(w, v)
            
            tray_real.append(real.pose())
            
            # --- EKF: Localización ---
            # Aplicar ruido de proceso a P antes de la corrección
            P = P + Q
            
            radius = 3
            center = ideal.pose()[:2]
            P = localizacion_ekf(objetivos, real, ideal, P, Q, R_var, center, radius, mostrar_gui)
            
            tray_ideal.append(ideal.pose())
            
            if mostrar_gui:
                mostrar(objetivos, tray_ideal, tray_real)
            
            espacio += v
            tiempo += 1
            
        # Fin de trayecto a este punto -> Guardar error baliza
        distanciaObjetivos.append(distancia(tray_real[-1], punto))

    toc = time.time()
    
    # Calcular desviación de trayectorias
    desviacion = np.sum(np.abs(np.subtract(tray_real, tray_ideal)))
    
    # Imprimir resultados finales (como antes)
    if len(tray_ideal) > 1000:
        print("<!> Trayectoria muy larga ⇒ quizás no alcanzada posición final.")
    
    print(f"Recorrido: {espacio:.3f}m / {tiempo/FPS:.3f}s")
    print(f"Distancia real al objetivo final: {distanciaObjetivos[-1]:.3f}m")
    
    # Mostrar error por cada baliza
    print("Distancias a cada baliza:")
    for i, err in enumerate(distanciaObjetivos):
        print(f"  Baliza {i+1}: {err:.3f}m")
    
    print(f"Suma de distancias a objetivos: {np.sum(distanciaObjetivos):.3f}m")
    print(f"Tiempo real invertido: {toc-tic:.3f}sg")
    print(f"Desviacion de las trayectorias: {desviacion:.3f}")
    
    if mostrar_gui:
        mostrar(objetivos, tray_ideal, tray_real)
        input("Presiona Enter para continuar...")
        plt.close('all')
    
    print(f"Resumen: {toc-tic:.3f} {desviacion:.3f} {np.sum(distanciaObjetivos):.3f}")

# Bloque main original por si se ejecuta el archivo directamente
if __name__ == "__main__":
    # Valores por defecto para prueba rápida
    objs = [[1,3], [2,2], [0,0]]
    pose = [0, 0, 0]
    noise = {'lin': 0.1, 'ang': 0.1, 'sense': 0.1}
    ejecutar_simulacion(objs, pose, noise, mostrar_gui=True)
