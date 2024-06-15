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
            'A': (1.5,.5), 'B': (4.6, 1.6), 'C': (6.4,2.2), 'D': (10.6, 2.4), 
            'E': (2,2.6), 'F': (.7,3.2), 'G': (3.5,4.5), 'H': (5, 5), 'I': (11, 5),
            'J': (11.1, 6), 'K': (.5,5.9), 'L': (2.8, 6), 'M': (4.6, 6.4), 'N': (5.8 ,6.9),
            'O': (7.3,7.3), 'P': (4.9, 8.8), 'Q': (9.9,6.5), 'R': (5.8 ,9.3), 'S': (10.2,9.1)
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
for node in ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S']:
    game_map.add_node(node)

#Agregamos conexiones entre nodos y sus pesos.
game_map.add_edge('A', 'B', 3)

game_map.add_edge('B', 'A', 3)
game_map.add_edge('B', 'G', 3)
game_map.add_edge('B', 'C', 2)

game_map.add_edge('C', 'B', 2)
game_map.add_edge('C', 'H', 3)
game_map.add_edge('C', 'I', 5)

game_map.add_edge('D', 'I', 3)

game_map.add_edge('E', 'F', 1)

game_map.add_edge('F', 'E', 1)
game_map.add_edge('F', 'G', 3)

game_map.add_edge('G', 'F', 3)
game_map.add_edge('G', 'B', 3)
game_map.add_edge('G', 'H', 1)
game_map.add_edge('G', 'L', 1)

game_map.add_edge('H', 'M', 1)
game_map.add_edge('H', 'G', 1)
game_map.add_edge('H', 'C', 3)

game_map.add_edge('I', 'C', 5)
game_map.add_edge('I', 'D', 3)
game_map.add_edge('I', 'J', 1)
game_map.add_edge('I', 'Q', 2)

game_map.add_edge('J', 'I', 1)

game_map.add_edge('K', 'L', 2)

game_map.add_edge('L', 'K', 2)
game_map.add_edge('L', 'G', 1)
game_map.add_edge('L', 'M', 2)

game_map.add_edge('M', 'L', 2)
game_map.add_edge('M', 'H', 1)
game_map.add_edge('M', 'N', 1)

game_map.add_edge('N', 'M', 1)
game_map.add_edge('N', 'P', 3)
game_map.add_edge('N', 'O', 1)

game_map.add_edge('O', 'N', 1)
game_map.add_edge('O', 'R', 3)
game_map.add_edge('O', 'S', 4)
game_map.add_edge('O', 'Q', 3)

game_map.add_edge('P', 'R', 1)
game_map.add_edge('P', 'N', 3)

game_map.add_edge('Q', 'O', 3)
game_map.add_edge('Q', 'I', 2)

game_map.add_edge('R', 'P', 1)
game_map.add_edge('R', 'O', 3)

game_map.add_edge('S', 'O', 4)

#Calculamos el árbol de expansión mínima (Minimum Spanning Tree) usando Prim's Algorithm.
start_node = 'A'
total_cost, mst_edges = game_map.prim(start_node)
print("Costo total del Minimum Spanning Tree:", total_cost)

#Dibujamos el grafo con el Minimum Spanning Tree resaltado.
game_map.draw(mst_edges)