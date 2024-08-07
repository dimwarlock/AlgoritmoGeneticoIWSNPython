import random
import psycopg2
import time
from deap import base, creator, tools, algorithms
import numpy as np
import math
from pygame.locals import *
import networkx as nx
import matplotlib.pyplot as plt

conn = psycopg2.connect(
    host = "localhost", 
    database = "st", 
    user = "postgres",
    password = "1234",
    port = "5432" 
)

conn.autocommit = True





#   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   Algoritmo Genetico   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   #
#     /\---/\    
# _   | [] []|   ________
# ||__|___^__|  < Inicia |
# | _____ |      --------
# |_|   |_|
            #     /\---/\    
            # _   | [] ><|   _______
            # ||__|___^__|  < El... |
            # | _____ |      -------
            # |_|   |_|
                        #     /\---/\    
                        # _   | [] []|   ____________________
                        # ||__|___^__|  < Algoritmo Genetico!|
                        # | _____ |      --------------------
                        # |_|   |_|

# < < < < < < < < < < < < < < < <   Algoritmo que precalcula Posiciones de Transmisores para cada Sensor   < < < < < < < < < < < < < < < < #   

# Definicion de un Cursor como Variable Global para la Funcion del Algoritmo Genético.
cursor = conn.cursor()

# Definicion de variables importantes
PosicionesSensores=[]
LargoCableSensorTransmisor = 20
LargoConexionTransmisores = 1000
num_sensores = 20

# DEBUG - Codigo Raandom Antiguo 
# for n in range(1, (num_sensores + 1)):
    #Este Codigo es claramente inviable, ha de corregirse
#    PosicionesSensores.append(Puntos[n*(n + (5 * n))])

#Leer Archivo TXT con el los Puntos de los Sensores.
with open("PuntosSensores.txt") as fname:
    lineas = fname.readlines()
    for linea in lineas:
        PosicionesSensores.append(linea.strip('\n'))

# DEBUG mostrar arreglo con las Posiciones de los Sensores
# input(PosicionesSensores)


# Variable que contiene los Puntos de la Malla Triangular
Puntos = []

#Leer Archivo TXT con el los Puntos sueltos.
with open("Puntos.txt") as fname:
    lineas = fname.readlines()
    for linea in lineas:
        Puntos.append(linea.strip('\n'))

# Función para calcular la distancia entre un transmisor y un sensor
def distancia(transmisor, sensor):
    x1 = float(transmisor.split()[0])
    y1 = float(transmisor.split()[1])
    z1 = float(transmisor.split()[2])

    x2 = float(sensor.split()[0])
    y2 = float(sensor.split()[1])
    z2 = float(sensor.split()[2])

    distancia = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

    return distancia

# Funcion que obtiene los Transmisores posibles para c/Sensor
def obtenerPosicionesTransmisoresPorSensor(Puntos, PosicionesSensores):
    puntos_cercanos = [[] for _ in range(len(PosicionesSensores))]  # Inicializamos una lista vacía para cada sensor

    for i, sensor in enumerate(PosicionesSensores):
        print(i)
        for punto in Puntos:    
            if distancia(punto, sensor) <= LargoCableSensorTransmisor:
                puntos_cercanos[i].append(punto)

    return puntos_cercanos

# Inicio de algoritmo para calcular posiciones de Transmisores
TransmisoresPosibles = obtenerPosicionesTransmisoresPorSensor(Puntos, PosicionesSensores)

# DEBUG - Imprimir los puntos cercanos a cada sensor
# for i, puntos_sensor in enumerate(TransmisoresPosibles):
#     print(f"Puntos cercanos al sensor {PosicionesSensores[i]}: {puntos_sensor}\n")

# input("El Programa ha finalizado, si se desea continuar, pulsar Enter.")



# < < < < < < < < < < < < < < < <   Funciones para el Algoritmo Genetico   < < < < < < < < < < < < < < < < # 

# Codigo que genera un Grafo con un Individuo que tiene una conexion Transmisor para c/Sensor

# Funcion para generar una matriz que tiene los Puntos de los Transmisores
def GenerarMatriz(Individuo):
    import numpy as np

    # Crear una matriz de 50 listas, donde cada lista contiene un una ubicacion de un Sensor en formato Float, para poder generar el Grafo
    puntos_seleccionados = []
    # Almacenar los Puntos en formato Coordenadas en el Arreglo puntos_seleccionados
    for i in range(len(Individuo)):
        punto = TransmisoresPosibles[i][Individuo[i]]
        puntos_seleccionados.append(punto)

    # DEBUG - Imprimir Puntos Seleccionados
    # input(puntos_seleccionados)

    # Convertir la lista de puntos seleccionados a un array de NumPy con dtype=float
    matriz = np.array([punto.split() for punto in puntos_seleccionados], dtype=float)
    
    return matriz



# Funcion para Generar Grafo con los Transmisores Elegidos
def GenerarGrafo(Individuo):    
    matriz = GenerarMatriz(Individuo)
    
    # DEBUG - Imprimir Matriz
    # print(matriz)
    # input("Asi se ve la primera matriz.")

    # Crear el grafo
    grafo = nx.Graph()
    # Agregar nodos al grafo
    num_nodos = len(matriz)
    grafo.add_nodes_from(range(num_nodos))

    # Calcular las distancias y agregar las aristas en el grafo 3D
    for i in range(num_nodos):
        for j in range(i + 1, num_nodos):
            distancia = np.linalg.norm(matriz[i] - matriz[j])  # Calcular distancia euclidiana
            if distancia <= LargoConexionTransmisores:  # Límite de distancia de conexión
                grafo.add_edge(i, j, weight=distancia)
    
    # DEBUG - Visualizar Grafo de Todas las Conexiones Posibles

    # Proyectar el grafo a 2D para visualización
    #pos_2d = {i: matriz[i, :2] for i in range(num_nodos)}

    # Crear el grafo 2D a partir del grafo 3D y las posiciones proyectadas
    #grafo_2d = nx.Graph(grafo)
    #nx.set_node_attributes(grafo_2d, pos_2d, 'pos')

    # Dibujar el grafo 2D
    #nx.draw(grafo_2d, pos_2d, with_labels=True, node_size=500, font_size=10)
    #labels = nx.get_edge_attributes(grafo_2d, 'weight')
    #nx.draw_networkx_edge_labels(grafo_2d, pos_2d, edge_labels=labels)
    #plt.show()

    return grafo



# Esta funcion genera el Grafo con solo las Conexiones Validas y su Arbol de Cobertura minima que es retornado
def GenerarArbolConexionesValidas(ConexionesValidas):
    # Almacenar los puntos de las Conexiones Validas en un arreglo Puntos Validos, se inicia con un Conjunto Set para evitar puntos duplicados
    PuntosValidosSet = set()
    for Conexion in ConexionesValidas:
        PuntosValidosSet.add(str(Conexion[0]).strip("[]").replace(",", "").strip("'"))
        PuntosValidosSet.add(str(Conexion[1]).strip("[]").replace(",", "").strip("'"))

    # Arreglo de Puntos Validos
    PuntosValidos = list(PuntosValidosSet)

    # Convertir la lista de PuntosValidos a un array de NumPy con dtype=float
    matriz = np.array([punto.split() for punto in PuntosValidos], dtype=float)

    # Crear el grafo
    grafo = nx.Graph()

    # Agregar nodos al grafo
    num_nodos = len(matriz)
    grafo.add_nodes_from(range(num_nodos))

    # Agregar nodos y aristas al grafo
    for Linea in ConexionesValidas:
        Punto1, Punto2 = Linea

        Punto1Dato = str(Punto1).strip("[]").replace(",", "")
        Punto2Dato = str(Punto2).strip("[]").replace(",", "")

        Punto1Float = np.array([Punto1Dato.split()], dtype=float)
        Punto2Float = np.array([Punto2Dato.split()], dtype=float)

        distancia = np.linalg.norm(Punto1Float - Punto2Float)
        Index1 = PuntosValidos.index(Punto1Dato)
        Index2 = PuntosValidos.index(Punto2Dato)
        grafo.add_edge(Index1, Index2, weight=distancia)

    # DEBUG - Visualizar Grafo de Solo Conexiones Validas
    # Proyectar el grafo a 2D para visualización
    #pos_2d = {i: matriz[i, :2] for i in range(num_nodos)}

    # Crear el grafo 2D a partir del grafo 3D y las posiciones proyectadas
    #grafo_2d = nx.Graph(grafo)
    #nx.set_node_attributes(grafo_2d, pos_2d, 'pos')

    # Dibujar el grafo 2D
    #nx.draw(grafo_2d, pos_2d, with_labels=True, node_size=500, font_size=10)
    #labels = nx.get_edge_attributes(grafo_2d, 'weight')
    #nx.draw_networkx_edge_labels(grafo_2d, pos_2d, edge_labels=labels)
    #plt.show()


    # Generar Arbol de cobertura minima
    ArbolDeCobertura = nx.minimum_spanning_tree(grafo, weight='weight')

    # Pregunta si es un Arbol de Covertura ideal que cubre todos los Transmisores
    if int(ArbolDeCobertura.size()) == (int(num_sensores) - 1):
        # DEBUG - Ver si son iguales
        #print("Cantidad de Lineas en el Arbol: " + str(ArbolDeCobertura.size()))
        #print("Cantidad de Sensores :" + str(int(num_sensores) - 1))
        
        # Si cubre todos los Transimsores, se visualiza
        pos_2d = {i: matriz[i, :2] for i in range(num_nodos)}
        nx.draw(ArbolDeCobertura, pos_2d, with_labels=True, node_color = "red", node_size=500, font_size=10)
        labels = nx.get_edge_attributes(ArbolDeCobertura, 'weight')
        nx.draw_networkx_edge_labels(ArbolDeCobertura, pos_2d, edge_labels=labels)
        plt.show()
        return ArbolDeCobertura
    else:
        return ArbolDeCobertura


# Declarar Diccionario de Lineas
LineasInterseccionesCalculadas = {}

# Función que evalua la interseccion de cada conexion Transmisor/Sensor
def ComprobarInterseccionConexionesYGenerarArbol(Individuo):
    Grafo = GenerarGrafo(Individuo)
    
    matriz = GenerarMatriz(Individuo)

    # DEBUG - Imprimir Grafo en Consola
    # input(Grafo.edges(data='weight'))

    # Crear un arreglo de líneas representando las aristas del grafo
    lineas_arreglo = []
    for nodo_inicio, nodo_fin, _ in Grafo.edges(data='weight'):
        linea = [matriz[nodo_inicio].tolist(), matriz[nodo_fin].tolist()]
        lineas_arreglo.append(linea)

    # DEBUG - Ver las Conexiones
    # print(lineas_arreglo)
    # input("El Programa ha finalizado.")

    # Arreglo que ha de guardar las Conexiones que no intersectan
    ConexionesValidas = []


    # Convertir las Lineas a LineString y evaluar su interseccion, si no intersecta se guarda en un Arreglo LineasValidas para generar un nuevo Grafo
    for Conexion in lineas_arreglo:
        P1 = Conexion[0]
        P2 = Conexion[1]
        
        # DEBUG - Mostrar P1 y P2
        # print(P1)
        # print(P2)
        # input("Asi se ven los puntos de una linea, finalizar programa.")

        # Calcular Interseccion de P1 y P2 por si solos y guardar la suma de sus valores en un Diccionario.
        TuplaLinea = tuple(P1 + P2)

        # Preguntar si esta combinacion de puntos ya fue calculada, si no, calcular y almacenar en Diccionario
        if TuplaLinea in LineasInterseccionesCalculadas:
            # Si esta linea ya fue calculada, solo se toma el valor de sus Intersecciones del Diccionario
            InterseccionesReales = LineasInterseccionesCalculadas[TuplaLinea]

            # DEBUG - Evidencia de que funciona el almacenamiento de lineas vistas anteriormente
            #print("Ya he visto esta linea anteriormente, tiene un valor de: " + str(InterseccionesReales))
        else:
            # Calcular Intersecciones
            punto_1 = "'POINT(" + str(P1).strip("[]").replace(",", " ") + ")'"
            query = "SELECT count (*) FROM Triangulos WHERE ST_3DIntersects(" + punto_1 + "::geometry, triangulo);"
            cursor.execute(query)
            NumeroInterseccionesP1 = cursor.fetchall()

            punto_2 = "'POINT(" + str(P2).strip("[]").replace(",", " ") + ")'"
            query = "SELECT count (*) FROM Triangulos WHERE ST_3DIntersects(" + punto_2 + "::geometry, triangulo);"
            cursor.execute(query)
            NumeroInterseccionesP2 = cursor.fetchall()

            # Convertir las intersecciones a Numeros
            InterseccionesP1 = str(NumeroInterseccionesP1).strip("[]").strip("()").strip(",")
            #input(InterseccionesP1)

            InterseccionesP2 = str(NumeroInterseccionesP2).strip("[]").strip("()").strip(",")
            #input(InterseccionesP2)

            # Calcular suma de Intersecciones y almacenar en Diccionario
            InterseccionesValidas = int(InterseccionesP1) + int(InterseccionesP2)
            
            # PEOR CASO - No se consideran las Intersecciones Validas
            #InterseccionesValidas = 0

            # Generar Linea
            linea_actual = "'LINESTRING(" + str(P1).strip("[]").replace(",", " ") + ", " + str(P2).strip("[]").replace(",", " ") + ")'"

            # DEBUG - Sintaxis Linestring
            # print(linea_actual)
            # input("Asi se ve mi Linestring.")


            # Evaluar la interseccion de la Linea en la Malla Triangular
            query = "SELECT count (*) FROM Triangulos WHERE ST_3DIntersects(" + linea_actual + "::geometry, triangulo);"
            cursor.execute(query)


            # EXPERIMENTAL - Evaluar la interseccion de la Linea con los Triangulos que estan en el rango de la Linea
            #query = "SELECT COUNT (*) FROM Triangulos WHERE ST_3DIntersects(" + linea_actual + "::geometry, triangulo) AND ST_3DDWithin(" + linea_actual + "::geometry, triangulo, 0);"
            #cursor.execute(query)

            triangulosIntersectados = cursor.fetchall()

            # Restar la cantidad de InterseccionesValidas a las Intersecciones obtenidas para obtener las Intersecciones reales
            triangulosIntersectadosINT = int(str(triangulosIntersectados).strip("[]").strip("()").strip(","))
            InterseccionesReales = triangulosIntersectadosINT - InterseccionesValidas
            #input(InterseccionesReales)

            # Almacenar InterseccionesReales en Diccionario
            LineasInterseccionesCalculadas[TuplaLinea] = InterseccionesReales

        # Si no hay Interseccion en la Conexion Transmisor/Sensor, se considera para el Arbol de Cobertura Minima
        if (InterseccionesReales == 0):
            #print(triangulosIntersectados)
            #print("La Linea no intersecta nada.")
            ConexionesValidas.append(Conexion)
        #else:
            #print("Intersecta la siguiente cantidad de veces: " + str(triangulosIntersectados))
        # DEBUG - Probar con individuos de 13 intersecciones como ejemplo para comprobar que, si hay Conexiones Validas, se usan correctamente para el Arbol de Cobertura.
        # elif (str(triangulosIntersectados) == "[(13,)]"):
        #    ConexionesValidas.append(Conexion)

    # DEBUG - Imprimir ConexionesValidas
    # print(ConexionesValidas)
    # input("Esas son las conexiones validas, finalizar programa.")

    # Si no hay conexiones Validas, se retorna 0
    if (ConexionesValidas == []):
        return 0
    else:
        # Se crea un nuevo Grafo con las Conexiones que demostraron ser válidas y su Arbol de Cobertura minima
        ArbolDeCobertura = GenerarArbolConexionesValidas(ConexionesValidas)        
        return ArbolDeCobertura


# Función para calcular la isla más grande en el árbol de cobertura
def CalcularIslaMasGrande(arbol_cobertura):
    componentes_conectados = list(nx.connected_components(arbol_cobertura))
    isla_mas_grande = max(componentes_conectados, key=len)
    tamano_isla_mas_grande = len(isla_mas_grande)
    return tamano_isla_mas_grande


# Funcion de Evaluación del Individuo
def evaluar_individuo(Individuo):
    # DEBUG - Visualizacion de formato de Individuo
    # print("Asi se ve el Individuo:")
    # print(Individuo)
    # input("El programa ha finalizado, detener ejecucion.")

    # Variable Fitness
    Fitness = 0

    # Variable que almacena el Arbol de Cobertura, si no hay conexiones, su valor es 0
    ArbolDeCobertura =  ComprobarInterseccionConexionesYGenerarArbol(Individuo)
   
    # Si no hay Conexiones Válidas, el Fitness es 0
    if (ArbolDeCobertura == 0):
        # input ("El Fitness fue 0, que mal.")
        return Fitness,
    else:
    # Si hay Conexiones Válidas y por lo tanto ya hay un Arbol de Cobertura Minima retornado por la Funcion, se procede a ver la Isla mas grande para el Fitness
        FitnessFiltro = CalcularIslaMasGrande(ArbolDeCobertura)
        
        if (FitnessFiltro == num_sensores):
            return 1,

        # Mientras mas alto el valor de la Isla mas grande, mas cerca estara el Fitness de 1
        Fitness = 1 - (1/FitnessFiltro)
        
        return Fitness,



# Crear la configuración de DEAP

# Número de elementos en el individuo
num_elementos = num_sensores

creator.create("FitnessMax", base.Fitness, weights=(1.0,))
creator.create("Individual", list, fitness=creator.FitnessMax)

toolbox = base.Toolbox()

def crear_elemento(index):
    return random.randint(0, len(TransmisoresPosibles[index]) - 1)

def crear_individual():
    return [crear_elemento(i) for i in range(num_elementos)]

def mutar_individuo(individuo, indpb):
    if random.random() < indpb:
        indice = random.randint(0, len(individuo) - 1)
        nuevo_valor = crear_elemento(indice)
        individuo[indice] = nuevo_valor
    return individuo,


toolbox.register("individual", tools.initIterate, creator.Individual, crear_individual)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)
toolbox.register("evaluate", evaluar_individuo)
toolbox.register("mate", tools.cxTwoPoint)
toolbox.register("mutate", mutar_individuo, indpb=0.2)
toolbox.register("select", tools.selTournament, tournsize=5)

# Parámetros del algoritmo genético
tamano_poblacion = num_sensores * 5
num_generaciones = num_sensores * 5

# Crear la población inicial
poblacion = toolbox.population(n=tamano_poblacion)

# Diccionario para almacenar los fitness de los individuos por generación
fitness_dict = {}

# DEBUG - Cantidad de veces que se ejecuto evaluacion en la Generacion
#numero_evaluaciones = 0

# Variable que indica si ya existe un mejor individuo para no tener que seguir generando Generaciones
ExisteMejorIndividuo = False

# DEBUG - Medir tiempo
inicio = time.time()

# Evolucionar la población
for generacion in range(num_generaciones):
    #print("Generacion nueva")
    offspring = algorithms.varAnd(poblacion, toolbox, cxpb=0.8, mutpb=0.05)

    # Si ya existe el individuo ideal, detener la generacion de Generaciones
    if ExisteMejorIndividuo == True:
        break

    for ind in offspring:
        # Convierte la lista en una tupla para usarla como clave en el diccionario
        ind_tuple = tuple(ind)
        
        # Verifica si el individuo ya tiene un fitness almacenado
        if ind_tuple in fitness_dict:
            # Si el fitness está almacenado, se transfiere, es decir, si el individuo es uno que estaba en una generacion anterior, copia su fitness
            ind.fitness.values = fitness_dict[ind_tuple]
            #print("Ya vi este individuo antes, tiene un fitness de: " + str(ind.fitness.values))
        else:
            # Si no hay un fitness almacenado, calcula y almacena el nuevo fitness
            fitness = toolbox.evaluate(ind)
            ind.fitness.values = fitness
            fitness_dict[ind_tuple] = fitness
            #numero_evaluaciones = numero_evaluaciones + 1
        
        # Pregunta si antes de terminar se encontro un Individuo Ideal que tiene el Fitness Maximo y por lo tanto conecta todos los Sensores
        if str(ind.fitness.values) == "(1.0,)":
            print("Se ha encontrado un Individuo que conecta todos los Transmisores como Mejor Individuo.")
            ExisteMejorIndividuo = True;
            break

    #print("El numero de evaluaciones de esta generacion fue de: " + str(numero_evaluaciones))
    
    poblacion = offspring
    
    numero_evaluaciones = 0

# DEBUG - Medir tiempo
fin = time.time()

print(fin-inicio)

# Obtener el mejor individuo después de las generaciones
mejor_individuo = tools.selBest(poblacion, k=1)[0]
print("Mejor individuo:", mejor_individuo)

# DEBUG - Cantidad de Individuos evaluados
#print("He evaluado esta cantidad de Individuos: " + str(len(fitness_dict)))