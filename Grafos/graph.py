from queue import PriorityQueue

class Graph():
    #Spawn del grafo
    def __init__(self, n: int, directed = False) -> None:
        self.n = n
        self.E = 0
        self.directed = directed
        self.adj_list: list[list[tuple]] = [[] for _ in range(n)]
        
    def add_edge(self, v1: int, v2: int, weight: int = 1) -> bool:
        if 0 <= v1 <= self.n-1 and 0 <= v2 <= self.n-1:
            self.adj_list[v1].append((v2, weight))
            self.adj_list[v1].sort()
            self.E += 1
            #Caso de bucle para que no se repita la arista
            if v1 == v2:
                return True
            #Caso base
            elif not self.directed:
                self.adj_list[v2].append((v1, weight))
                self.adj_list[v2].sort()
            self.E += 1
            return True
        return False
    
    #Propiedades del grafo
    def degree(self, vertex: int) -> tuple:
        exit_deg = len(self.adj_list[vertex])
        entry_deg = 0
        if self.directed:
            for adj in self.adj_list:
                for edge in adj:
                    if vertex == edge[0]:
                        entry_deg += 1
        else:
            entry_deg = exit_deg
        return exit_deg, entry_deg
    
    def deg_sequence(self) -> list: 
        out = [self.degree(em) for em in range(self.n)]
        out.sort(reverse=True)
        return out
    
    def max_degree(self) -> tuple:
        seq = self.deg_sequence()
        return max([seq[i][0] for i in range(self.n)]), max([seq[i][1] for i in range(self.n)])
    
    def min_degree(self) -> tuple:
        seq = self.deg_sequence()
        return min([seq[i][0] for i in range(self.n)]), min([seq[i][1] for i in range(self.n)])
    
    def __num_of_components(self, vertex: int, visited: list = None) -> list:
        #Incializar visited del dfs
        if visited is None:
            visited = [False] * self.n
        #Ejecutar recorrido dfs
        visited[vertex] = True
        for em in self.adj_list[vertex]:
            if not visited[em[0]]:
                visited = self.__num_of_components(em[0], visited)
        return visited
    
    def num_of_components(self) -> int:
        components = 1
        visited = self.__num_of_components(0)
        #Si no todos los vertices han sido visitados, hay más de un componente
        while not all(visited):
            #Buscar el primer nodo no visitado
            i = 0
            while visited[i]:
                i += 1
            #DFS desde el primer no visitado
            new_visited = self.__num_of_components(i)
            #Actualizar visited y aumentar el numero de componentes
            for index in range(self.n):
                if (not visited[index]) and new_visited[index]:
                    visited[index] = True
            components += 1
        return components
        
    def is_connected(self) -> bool:
        return True if self.num_of_components() == 1 else False
    
    def is_eulerian(self) -> bool:
        for vertex in range(self.n):
            if not(self.degree(vertex)[0] % 2 == 0):
                return False
        if (self.is_connected()):
            return True
        return False
            
    def is_semieulerian(self) -> bool:
        even_degrees = 0
        odd_degrees = 0
        for vertex in range(self.n):
            if (self.degree(vertex)[0] % 2 == 0):
                even_degrees += 1
            else:
                odd_degrees += 1
                if odd_degrees > 2:
                    return False 
        return True if (self.is_connected() and odd_degrees == 2) else False
    
    def is_r_regular(self, r: int) -> bool:
        state = False
        for em in range(self.n):
            if (self.degree(em) == r ):
                state = True
            else:
                state = False
        return state
    
    def is_complete(self) -> bool:
        return self.is_r_regular(self.n - 1)
    
    def has_loops(self) -> bool:
        for i in range(self.n):
            if (i in self.adj_list[i][0]):
                return False
            else:
                state = True
        return state
    
    def has_cycles(self) -> bool:
        pass
    
    #Recorridos
    def bfs(self, vertex: int) -> None:
        visited = [False] * self.n
        queue = [vertex]
        visited[vertex] = True
        while len(queue) > 0:
            pointer = queue.pop(0)
            print(pointer, end = " ")
            for em in self.adj_list[pointer]:
                if not visited[em[0]]:
                    queue.append(em[0])
                    visited[em[0]] = True
        print()
        
    def __dfs (self, vertex: int, visited: list = None) -> list:
        if visited == None:
            visited = [False] * self.n
            
        visited[vertex] = True
        print(vertex , end = " ")
        for em in self.adj_list[vertex]:
            if not visited[em[0]]:
                visited = self.__dfs(em[0], visited)
        return visited
            
    def dfs(self, vertex: int) -> None:
        self.__dfs(vertex)
        print()
    
    #Caminos minimos
    def dijkstra (self, start_vertex: int) -> tuple:
        #Inicializar matrices
        C = [[0] * self.n] * self.n
        for index, adjs in enumerate(self.adj_list):
            for em in adjs:
                C[index][em[0]] = em[1]
                if not self.directed:
                    C[em[0]][index] = em[1]
        print(C)
        '''
        D = [float("inf")] * self.n
        pad = [None] * self.n
        visit = [False] * self.n
        D[start_vertex] = 0
        
        while (not all(visit)):
            #Encontrar el primer vertice no visitado
            for i in range (self.n):
                if not visit[i]:
                    min_not_visited = i
                    break
            #marcar al elemento no visitado de menor peso
            for i in range (self.n):
                if not visit[i] and D[i] < D[min_not_visited]:
                    min_not_visited = i
            v = min_not_visited
            visit[v] = True
            #Actualizar desde v
            for i in self.adj_list[v]:
                if (D[v] + self.C[v][i] < D[i] and not visit[i]):
                    D[i] = D[v] + self.C[v][i]
                    pad[i] = v
        return D, pad
        '''
        
    def floyd_warshall(self):
        #Crear matriz de distancia D
        D = self.cost_matrix
        for i in range(self.n):
            for j in range(self.n):
                if i != j and D[i][j] == 0:
                    D[i][j] = float("inf")
        #Crear matriz de caminos P
        P = [[0] * self.n for _ in range(self.n)]
        for i in range(self.n):
            for j in range(self.n):
                P[i][j] = j
        #Recorridos
        for k in range(self.n):
            for i in range(self.n):
                for j in range(self.n):
                    if D[i][k] + D[k][j] < D[i][j]:
                        D[i][j] = D[i][k] + D[k][j]
                        P[i][j] = k
        return D, P
        
    #Arboles de expansion minima
    def kruskal(self):
        q = PriorityQueue()
        #Organizar las aristas
        E = []
        for v, adj_list_v in enumerate(self.adj_list):
            for em in adj_list_v:
                pass
        E.append()
        #Añadir al grafo resultante todas las aristas que no formen ciclos
        #Termina si el resultante tiene n-1
        pass
        '''
        for edge in self.E():
            q.put((w(edge), edge))
        out = Graph(n, self.directed)
        while len(E(out)) < self.n - self.num_of_components() and not q.empty:
            pointer = q.get()
            out.add_edge(pointer)
            if not out.has_cycles:
                out.delete_edge(pointer)
        return out    
        
        '''
    
    def prim(self, start_vertex):
        #En una cola de prioridades añadir todos los adyacentes a start_vertex
        #Añadir a la lista de vertices visitados start_vertex
        #Inicializar la lista de aritas de salida
        out_edges = []
        #Mientras que no se hayan visitado todos los vertices del grafo y la cola no esté vacía
            #Sacamos la arista de menor peso
            #Si el vertice final de la arista no ha sido visitada
                #Añadirla a los verices visitados
                #Añade la arista a las de salida
                #Añadir a la cola todas las aristas adyacentes al vertice final
        #retornar out
        pass
        '''
        q = priorityQueue
        for em in self.adj_list[start_vertex]
            q.put((w(em), (star_vertex, em)))
        Vertex, Edges = [start_vertex], []
        while len(Vertex) < G.n and not q.empty():
            (vi,vf) = q.get()
            if (vf not in Vertex):
                Vertex.append(vf)
        '''
    
    #Flujo máximo
    def ford_fulkerson(self, first_vertex: int, last_vertex: int):
        '''
        Inicializa los flujos
        Mientras haya caminos de aumento
            Calcular un camino de aumento
            Calcular el cuello de botella
            Para cada vertice en el camino de aumento
                Actualizar su flujo
        Retornar flujo para cada verice en el grafo
        '''
        pass
    
if __name__ == '__main__':
    #spawn
    test = Graph(4)
    test.add_edge(0, 1)
    test.add_edge(0, 2)
    test.add_edge(0, 3)
    test.add_edge(3, 1)
    
    print("=====Lista de adyacencia=====")
    print(test.adj_list)
    #recorridos
    print("=====Recorridos=====")
    print("DFS(0):")
    test.dfs(0)
    print("BFS(0):")
    test.bfs(0)
    #propiedades
    print("=====Propiedades=====")
    print("Degree sequence (exit, entry):")
    print(test.deg_sequence())
    print("max Degree (exit, entry):")
    print(test.max_degree())
    print("min Degree (exit, entry):")
    print(test.min_degree())
    print("Numero de componentes:")
    print(test.num_of_components())
    print("Es conexo:")
    print(test.is_connected())
    print("Es euleriano:")
    print(test.is_eulerian())
    print("Es semi-euleriano:")
    print(test.is_semieulerian())
    print("Es r-regular(2):")
    print(test.is_r_regular(2))
    print("Es completo:")
    print(test.is_complete())
    print("Tiene bucles:")
    print(test.has_loops())
    print("=====Caminos mínimos=====")
    print("Dijkstra")
    test.dijkstra(0)
    
    
    