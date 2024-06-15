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
            'A': (0,2.5), 'B': (2.1,2.2), 'C': (10.4,0), 'D': (8.5, 1.6), 
            'E': (7.8, 2.5), 'F': (10.1, 2.8), 'G': (13.8,2), 'H': (2.2, 3.5), 'I': (4, 3.5),
            'J': (5.4,3.6), 'K': (7.1, 3.5), 'L': (5.4, 4.4), 'M': (8.2, 5.1), 'N': (5.2, 5.8),
            'O': (12, 5.9), 'P': (2, 8.4), 'Q': (4.4, 7.9)
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
for node in ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q']:
    game_map.add_node(node)

#Agregamos conexiones entre nodos y sus pesos.
game_map.add_edge('A', 'B', 3)
game_map.add_edge('A', 'H', 3)

game_map.add_edge('B', 'A', 3)
game_map.add_edge('B', 'I', 4)
game_map.add_edge('B', 'C', 20)

game_map.add_edge('C', 'B', 20)
game_map.add_edge('C', 'D', 4)
game_map.add_edge('C', 'F', 6)

game_map.add_edge('D', 'C', 4)
game_map.add_edge('D', 'E', 1)

game_map.add_edge('E', 'D', 1)
game_map.add_edge('E', 'K', 1)

game_map.add_edge('F', 'C', 6)
game_map.add_edge('F', 'G', 6)
game_map.add_edge('F', 'O', 7)
game_map.add_edge('F', 'K', 5)

game_map.add_edge('G', 'F', 6)
game_map.add_edge('G', 'O', 10)

game_map.add_edge('H', 'A', 3)
game_map.add_edge('H', 'I', 3)
game_map.add_edge('H', 'P', 15)

game_map.add_edge('I', 'H', 3)
game_map.add_edge('I', 'B', 4)
game_map.add_edge('I', 'J', 1)

game_map.add_edge('J', 'I', 1)
game_map.add_edge('J', 'K', 2)
game_map.add_edge('J', 'L', 1)

game_map.add_edge('K', 'J', 2)
game_map.add_edge('K', 'M', 2)
game_map.add_edge('K', 'E', 1)
game_map.add_edge('K', 'F', 5)

game_map.add_edge('L', 'J', 1)
game_map.add_edge('L', 'N', 2)

game_map.add_edge('M', 'K', 2)
game_map.add_edge('M', 'O', 6)

game_map.add_edge('N', 'L', 2)
game_map.add_edge('N', 'Q', 3)

game_map.add_edge('O', 'Q', 20)
game_map.add_edge('O', 'M', 36)
game_map.add_edge('O', 'F', 7)
game_map.add_edge('O', 'G', 10)

game_map.add_edge('P', 'Q', 3)
game_map.add_edge('P', 'H', 15)

game_map.add_edge('Q', 'O', 20)
game_map.add_edge('Q', 'N', 3)
game_map.add_edge('Q', 'P', 3)


#Calculamos el árbol de expansión mínima (Minimum Spanning Tree) usando Prim's Algorithm.
start_node = 'A'
total_cost, mst_edges = game_map.prim(start_node)
print("Costo total del Minimum Spanning Tree:", total_cost)

#Dibujamos el grafo con el Minimum Spanning Tree resaltado.
game_map.draw(mst_edges)