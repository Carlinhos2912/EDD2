from queue import PriorityQueue

class Graph():
    def __init__(self, n: int = 1, directed = False) -> None:
        self.n = n
        self.directed = directed
        self.adj_list: dict = {f"{em}":[] for em in range(n)}
        
    def add
    def add_edge(self, v1, v2, weight = 1) -> bool:
        print(self.adj_list)
        if (v1 in self.adj_list.keys and v2 in self.adj_list.keys):
            #bucle
            if v1 == v2:
                self.adj_list[v1].append((v2, weight))
                print(self.adj_list)
                return True
            '''
            self.adj_list[v1].append(v2)
            self.adj_list[v1].sort()
            self.cost_matrix[v1][v2] = weight
            if not self.directed:
                self.adj_list[v2].append(v1)
                self.adj_list[v2].sort()
                self.cost_matrix[v2][v1] = weight
            return True
            '''
        return False
    
    def generate_edge_list (self) -> list:
        edges = []
        return edges
    
    #Propiedades del grafo
    def degree(self, vertex) -> tuple:
        exit_deg = len(self.adj_list[vertex])
        if self.directed:
            if (vertex in self.adj_list[i] for i in range(self.n)):
                entry_deg += 1
            return entry_deg, exit_deg
        return None, exit_deg
    
    def max_degree(self) -> tuple:
        if self.directed:
            return max(self.deg_sequence()[0]), max(self.deg_sequence()[1]) 
        else:
            return None, max(self.deg_sequence())
    
    def min_degree(self) -> tuple:
        if self.directed:
            return min(self.deg_sequence()[0]), min(self.deg_sequence()[1])    
        else:
            return None, min(self.deg_sequence())
    
    def deg_sequence(self) -> tuple: 
        exit_seq = [self.degree(i)[1] for i in range(self.n)]
        exit_seq.sort()
        if self.directed:
            entry_seq = [self.degree(i)[0] for i in range(self.n)]
            entry_seq.sort()
            return entry_seq, exit_seq
        return exit_seq
    
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
    
    
    
if __name__ == '__main__':
    test = Graph(4)
    
    test.add_edge()
    print(test.adj_list)
    