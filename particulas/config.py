#! /usr/bin/env python

from math import pi

P_INICIAL = [0., 4., 0.]
V_LINEAL   = .7
V_ANGULAR  = 140.
FPS = 10.
HOLONOMICO = 0
GIROPARADO = 0
LONGITUD   = .1

EPSILON = .15
V = V_LINEAL / FPS
W = V_ANGULAR * pi / (180*FPS)
NOISE = [0.01, 0.01, 0.01] 

N_MIN      = 100
N_MAX      = 1000
N_INICIAL  = N_MAX
FACTOR_DIN = 1500

UMBRAL_BUCLE   = 7.
UMBRAL_CALIDAD = 0.02

CENTER = [2, 2]
RADIUS  = 3 
