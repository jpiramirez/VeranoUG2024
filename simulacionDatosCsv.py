"""
Código para ejecutar algoritmo elegido y crear archivo de resultados
Determinar si se quiere tomar en cuenta o no obstaculos mediante la
variable "obstaculos"
"""

import csv
from pruebasAstargridMultiple import algorithm

nombre_archivo = "test_info_sin_obstaculos.csv"
obstaculos = False

distancia_minima = 10000
tiempo_total = 0
velocidad = 0.3


with open(nombre_archivo, mode='w', newline='') as archivo:
  escritor_csv = csv.writer(archivo)
  escritor_csv.writerow(["velocidad robots", f'{velocidad} m/s'])
  for i in range(30):
    
    tabla, d_min = algorithm(i, 7, obstaculos)
    datos = [["Robot", "Inicio", "Meta", "Distancia"]]

    datos = datos + tabla

    escritor_csv.writerow(["", f'SIMULACIÓN {i}', ""])
    escritor_csv.writerows(datos)
    if obstaculos:
      tiempo_total = max([dis[3] for dis in tabla]) / velocidad
    else:
      tiempo_total = sum([dis[3] for dis in tabla]) / velocidad

    escritor_csv.writerow(["Tiempo de simulacion", f'{tiempo_total:.3f} seg'])
    escritor_csv.writerow([])

    if d_min<distancia_minima:
      distancia_minima = d_min

  escritor_csv.writerow([])
  escritor_csv.writerow([])
  escritor_csv.writerow(["Menor distancia", distancia_minima])


print(f"Archivo {nombre_archivo} creado exitosamente.")
