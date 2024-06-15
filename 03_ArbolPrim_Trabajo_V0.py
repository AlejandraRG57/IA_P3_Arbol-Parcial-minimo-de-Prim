#Alejandra Rodriguez Guevara 21310127 6E1

#El Árbol Parcial Mínimo (APM) de un grafo es un subgrafo que une todos los vértices 
# del grafo original con el menor peso total posible y sin crear ciclos. En otras palabras, 
# se trata de un conjunto de aristas que conecta todos los vértices del grafo, con el 
# peso más bajo posible, y que no forma ciclos.

import heapq #Importamos la biblioteca heapq para trabajar con colas de prioridad.
import matplotlib.pyplot as plt #Importamos la biblioteca matplotlib para gráficos.
import networkx as nx #Importamos la biblioteca networkx para trabajar con grafos.

#Definición de la clase Graph que representa un grafo.
class Graph:
    def __init__(self):
        self.nodes = set() #Conjunto para almacenar los nodos del grafo.
        self.edges = {} #Diccionario para almacenar las aristas del grafo.

    #Método para agregar un nodo al grafo.
    def add_node(self, value):
        self.nodes.add(value) #Agregamos el nodo al conjunto de nodos.
        if value not in self.edges:
            self.edges[value] = {} #Inicializamos el diccionario de aristas para el nuevo nodo.

    #Método para agregar una arista al grafo.
    def add_edge(self, from_node, to_node, weight):
        self.edges[from_node][to_node] = weight #Agregamos la arista desde from_node a to_node con el peso dado.
        self.edges[to_node][from_node] = weight

    #Método para encontrar el árbol de expansión mínima usando el algoritmo de Prim.
    def prim(self, start):
        mst = set() #Conjunto para almacenar los vértices en el árbol de expansión mínima.
        mst_edges = [] #Lista para almacenar las aristas del árbol de expansión mínima.
        total_cost = 0 #Costo total del árbol de expansión mínima.

        visited = set() #Conjunto para almacenar los vértices visitados.
        priority_queue = [(0, start)] #Cola de prioridad para elegir la próxima arista a considerar.
        parent = {} #Diccionario para rastrear los padres de los nodos en el árbol de expansión mínima.

        while priority_queue:
            cost, node = heapq.heappop(priority_queue) #Sacamos el nodo con el menor costo.
            if node not in visited:
                visited.add(node) #Marcamos el nodo como visitado.
                total_cost += cost #Agregamos el costo de la arista al costo total.

                if node != start:
                    mst.add(node) #Agregamos el nodo al árbol de expansión mínima.
                    mst_edges.append((node, parent[node])) #Agregamos la arista al árbol de expansión mínima.

                #Recorremos los vecinos del nodo actual.
                for neighbor, weight in self.edges[node].items():
                    if neighbor not in visited:
                        heapq.heappush(priority_queue, (weight, neighbor)) #Agregamos el vecino a la cola de prioridad.
                        parent[neighbor] = node #Guardamos el nodo padre del vecino.

        return total_cost, mst_edges #Devolvemos el costo total y las aristas del árbol de expansión mínima.

    #Método para dibujar el árbol de expansión mínima.
    def draw(self, mst_edges):
        G = nx.Graph() #Creamos un objeto Graph de networkx.

        #Diccionario con las posiciones de los nodos para la visualización.
        positions = {
            'A': (10.5, 0), 'B': (9.8, 2), 'C': (10.8, 3.4), 'D': (7.6, 3), 
            'E': (7.8, 4.8), 'F': (4.1, 6.2), 'G': (0, 5.9), 'H': (.4, 2.8)
        }

        #Agregamos todos los nodos al grafo de networkx.
        for node in self.nodes:
            G.add_node(node)

        #Agregamos todas las aristas al grafo de networkx.
        for node, edges in self.edges.items():
            for neighbor, weight in edges.items():
                G.add_edge(node, neighbor, weight=weight)

        plt.figure(figsize=(10, 6))  #Configuramos el tamaño de la figura.
        #Dibujamos todos los nodos y aristas en color negro.
        nx.draw(G, pos=positions, with_labels=True, node_color='skyblue', node_size=1000, font_size=10, font_weight='bold')
        nx.draw_networkx_edges(G, pos=positions, edgelist=mst_edges, edge_color='r', width=2)  # Dibuja las aristas del árbol de expansión mínima en rojo.
        #Agregamos etiquetas de peso a las aristas.
        edge_labels = {(u, v): d['weight'] for u, v, d in G.edges(data=True)}
        nx.draw_networkx_edge_labels(G, pos=positions, edge_labels=edge_labels)
        plt.title("Minimum Spanning Tree (Prim's Algorithm)") #Título del gráfico.
        plt.show() #Mostramos el gráfico.

#Creamos el grafo.
game_map = Graph()

#Agregamos nodos.
for node in ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H']:
    game_map.add_node(node)

#Agregamos conexiones entre nodos y sus pesos.
game_map.add_edge('A', 'H', 20)
game_map.add_edge('A', 'G', 25)
game_map.add_edge('A', 'D', 5)
game_map.add_edge('A', 'B', 2)
game_map.add_edge('A', 'C', 4)

game_map.add_edge('B', 'A', 2)
game_map.add_edge('B', 'C', 1)
game_map.add_edge('B', 'E', 2)
game_map.add_edge('B', 'D', 2)
game_map.add_edge('B', 'H', 20)

game_map.add_edge('C', 'A', 4)
game_map.add_edge('C', 'B', 1)
game_map.add_edge('C', 'E', 5)

game_map.add_edge('D', 'A', 5)
game_map.add_edge('D', 'B', 2)
game_map.add_edge('D', 'E', 2)
game_map.add_edge('D', 'F', 7)
game_map.add_edge('D', 'G', 15)
game_map.add_edge('D', 'H', 15)

game_map.add_edge('E', 'C', 5)
game_map.add_edge('E', 'B', 2)
game_map.add_edge('E', 'D', 2)
game_map.add_edge('E', 'H', 15)
game_map.add_edge('E', 'F', 5)

game_map.add_edge('F', 'E', 5)
game_map.add_edge('F', 'D', 7)
game_map.add_edge('F', 'H', 10)
game_map.add_edge('F', 'G', 5)

game_map.add_edge('G', 'F', 5)
game_map.add_edge('G', 'D', 15)
game_map.add_edge('G', 'A', 25)
game_map.add_edge('G', 'H', 5)

game_map.add_edge('H', 'A', 20)
game_map.add_edge('H', 'B', 20)
game_map.add_edge('H', 'D', 15)
game_map.add_edge('H', 'E', 15)
game_map.add_edge('H', 'F', 10)
game_map.add_edge('H', 'G', 5)

#Calculamos el árbol de expansión mínima (Minimum Spanning Tree) usando Prim's Algorithm.
start_node = 'A'
total_cost, mst_edges = game_map.prim(start_node)
print("Costo total del Minimum Spanning Tree:", total_cost)

#Dibujamos el grafo con el Minimum Spanning Tree resaltado.
game_map.draw(mst_edges)