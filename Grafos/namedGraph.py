from queue import PriorityQueue

class NamedGraph():
    #Spawn del grafo
    def __init__(self, base_key, directed = False) -> None:
        self.n = 1
        self.directed = directed
        self.adj_list: dict = {base_key: []}
        
    def add_vertex(self, key) -> bool:
        if not key in self.adj_list.keys():
            self.adj_list[key] = []
            self.n += 1
            return True
        return False
        
    def add_edge(self, v1, v2, weight = 1) -> bool:
        if (v1 in self.adj_list.keys() and v2 in self.adj_list.keys()):
            if not v2 in self.adj_list[v1]:
                self.adj_list[v1].append((v2, weight))
                if not self.directed:
                    self.adj_list[v2].append((v1, weight))
                return True
        return False
    
    #Propiedades del grafo
    def degree(self, vertex) -> tuple:
        exit_deg = len(self.adj_list[vertex])
        if self.directed:
            for key in self.adj_list.keys():
                if (vertex in self.adj_list[key][0]):
                    entry_deg += 1
            return exit_deg, entry_deg
        return exit_deg, None
    
    def max_degree(self) -> tuple:
        if self.directed:
            return max(self.deg_sequence()[0]), max(self.deg_sequence()[1]) 
        else:
            return max(self.deg_sequence()[0]), None
    
    def min_degree(self) -> tuple:
        if self.directed:
            return min(self.deg_sequence()[0]), min(self.deg_sequence()[1])    
        else:
            return min(self.deg_sequence()[0]), None
    
    def deg_sequence(self) -> tuple: 
        exit_seq, entry_seq = [], []
        for key in self.adj_list.keys():
            exit_seq.append(self.degree(key)[0])
            if(self.directed):
                entry_seq.append(self.degree(key)[1])
        return exit_seq, entry_seq
    
    def r_num_of_components(self, vertex, visited = None) -> list:
        #Incializar visited del dfs
        if visited is None:
            visited = [False]*self.n
        #Ejecutar recorrido dfs
        visited[vertex] = True
        for em in self.adj_list[vertex]:
            if not visited[em]:
                visited = self.r_num_of_components(em, visited)
        return visited
    
    def num_of_components(self) -> int:
        components = 1
        visited = self.r_num_of_components(0)
        #Si no todos los vertices han sido visitados, hay más de un componente
        while not all(visited):
            #Buscar el primer nodo no visitado
            i = 0
            while visited[i]:
                i += 1
            #DFS desde el primer no visitado
            new_visited = self.r_num_of_components(i)
            #Actualizar visited y aumentar el numero de componentes
            for index in range(self.n):
                if (not visited[index]) and new_visited[index]:
                    visited[index] = True
            components += 1
        return components
        
    def is_connected(self) -> bool:
        return True if self.num_of_components() == 1 else False
    
    def is_eulerian(self) -> bool:
        pass
    
    def is_semieulerian(self) -> bool:
        pass
    
    def is_r_regular(self, r: int) -> bool:
        state = False
        for em in range(self.n):
            if (self.degree(em) == r ):
                state = True
            else:
                state = False
        return state
    
    def is_complete(self) -> bool:
        return True if self.is_r_regular(self.n - 1) else False
    
    def has_loops(self) -> bool:
        for i in range(self.n):
            if (i in self.adj_list[i]):
                return False
            else:
                state = True
        return state
    
    def has_cycles(self) -> bool:
        for v1 in range(self.n):
            for v2 in range(self.n):
                p1_2: list = self.path(v1, v2)
                p2_1: list = self.path(v2, v1)
                inverse_p2_1 = p2_1
                inverse_p2_1.reverse()
                #Si existen 2 caminos distintos entre v1 y v2
                if (len(p1_2) > 0 and len(p2_1) > 0) and p1_2 != inverse_p2_1:
                    return True
        return False
    
    #Recorridos
    def bfs(self, vertex) -> None:
        queue = []
        visited = [False] * self.n
        queue.append(vertex)
        visited[vertex] = True
        while len(queue) > 0:
            pointer = queue.pop(0)
            print(pointer, end = " ")
            for em in self.adj_list[pointer]:
                if not visited[em]:
                    queue.append(em)
                    visited[em] = True
        print()
        
    def dfs(self, vertex) -> None:
        if visited == None:
            visited = [False]*self.n
            
        visited[vertex] = True
        print(vertex , end = " ")
        for em in self.adj_list[vertex]:
            if not visited[em]:
                visited = self.dfs(em, visited)
        print()
        return visited
    
    #Caminos minimos
    def dijkstra (self, v_ini):
        D = [float("inf")] * self.n
        pad = [None] * self.n
        visit = [False] * self.n
        D[v_ini] = 0
        while (not all(visit)):
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
    def ford_fulkerson(self):
        pass
    
    
if __name__ == '__main__':
    print(chr(0 + 65))

    #spawn
    test = NamedGraph(base_key="Guatemala")
    test.add_vertex("Guatepeor")
    test.add_vertex("Mexico")
    test.add_vertex("El salvador")
    test.add_vertex("Polombia")
    test.add_edge("Guatemala", "Guatepeor")
    test.add_edge("Guatemala", "Polombia")
    test.add_edge("Guatemala", "El salvador")
    test.add_edge("El salvador", "Mexico")
    
    print("=====Lista de adyacencia=====")
    print(test.adj_list)
    print()
    #propiedades
    print("=====Propiedades=====")
    print("Degree sequence")
    print(tuple(test.deg_sequence()))
    print("max Degree")
    print(tuple(test.max_degree()))
    print("min Degree")
    print(tuple(test.min_degree()))
    print("Numero de componentes")
    print(test.num_of_components())
    
    