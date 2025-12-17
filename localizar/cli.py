#! /usr/bin/env python3

# cli.py
import sys
import os
import localizacion  # Importamos el módulo modificado

def imprimir_menu():
    print("\n")
    print("##############################################")
    print("#      SIMULADOR DE ROBOTS MÓVILES (CLI)     #")
    print("##############################################")
    print("1. Definir Nuevo Circuito (Balizas)")
    print("2. Configurar Parámetros de Ruido")
    print("3. Configurar Posición Inicial Robot")
    print("4. EJECUTAR SIMULACIÓN")
    print("5. Salir")
    print("----------------------------------------------")

def pedir_float(mensaje):
    while True:
        try:
            return float(input(mensaje))
        except ValueError:
            print("<!> Por favor, introduce un número válido.")

def opcion_circuito():
    print("\n--- Configuración del Circuito ---")
    while True:
        try:
            num = int(input("¿Cuántas balizas desea añadir? (Mínimo 3): "))
            if num >= 3:
                break
            print("<!> Debe haber al menos 3 balizas.")
        except ValueError:
            print("<!> Introduce un número entero.")
    
    nuevos_objetivos = []
    print(f"Introduce las coordenadas (X, Y) para las {num} balizas:")
    for i in range(num):
        print(f" > Baliza {i+1}:")
        x = pedir_float("   Coordenada X: ")
        y = pedir_float("   Coordenada Y: ")
        nuevos_objetivos.append([x, y])
    
    print("Circuito guardado correctamente.")
    input("Presiona Enter para continuar...")
    return nuevos_objetivos

def opcion_ruido(actuales):
    print("\n--- Configuración de Ruidos ---")
    print(f"Valores actuales: Lineal={actuales['lin']}, Angular={actuales['ang']}, Sensor={actuales['sense']}")
    
    n = {}
    n['lin'] = pedir_float("Nuevo ruido Lineal (ej. 0.01): ")
    n['ang'] = pedir_float("Nuevo ruido Angular (ej. 0.01): ")
    n['sense'] = pedir_float("Nuevo ruido Sensor (ej. 0.1): ")
    
    print("Parámetros de ruido actualizados.")
    input("Presiona Enter para continuar...")
    return n

def opcion_posicion():
    print("\n--- Configuración Posición Inicial ---")
    x = pedir_float("Posición X inicial: ")
    y = pedir_float("Posición Y inicial: ")
    th = pedir_float("Orientación inicial (radianes): ")
    
    print(f"Posición fijada en [{x}, {y}, {th}]")
    input("Presiona Enter para continuar...")
    return [x, y, th]

def main():
    # --- Valores por defecto ---
    objetivos = [[2, 2], [2, -2], [-2, -2], [-2, 2]] # Cuadrado básico
    pose_inicial = [0.0, 0.0, 0.0]
    ruidos = {'lin': 0.01, 'ang': 0.01, 'sense': 0.1}
    
    while True:
        imprimir_menu()
        print(f"Estado Actual -> Balizas: {len(objetivos)} | Ruido: {ruidos['lin']}/{ruidos['sense']} | Pose: {pose_inicial}")
        
        opcion = input("\nSelecciona una opción: ")
        
        if opcion == '1':
            objetivos = opcion_circuito()
        
        elif opcion == '2':
            ruidos = opcion_ruido(ruidos)
            
        elif opcion == '3':
            pose_inicial = opcion_posicion()
            
        elif opcion == '4':
            print("\nIniciando simulación...")
            # Preguntar si quiere ver gráficos (opcional, por defecto SÍ)
            graf = input("¿Mostrar gráficos en tiempo real? (s/n): ").lower().strip()
            mostrar = True if graf != 'n' else False
            
            try:
                localizacion.ejecutar_simulacion(objetivos, pose_inicial, ruidos, mostrar_gui=mostrar)
            except KeyboardInterrupt:
                print("\nSimulación interrumpida por el usuario.")
            except Exception as e:
                print(f"\n<!> Error durante la simulación: {e}")
                import traceback
                traceback.print_exc()
                input("Enter para continuar...")
                
        elif opcion == '5':
            print("Saliendo...")
            sys.exit(0)
        else:
            print("Opción no válida.")

if __name__ == "__main__":
    main()
