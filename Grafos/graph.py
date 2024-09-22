class Graph():
    def __init__(self, n: int , directed = False) -> None:
        self.n = n
        self.directed = directed
        self.adj: list[list[int]] = [[] for _ in range(n)]
        self.C = [[0] * n for _ in range(n)]
        
    def add_edge(self, v1, v2, weight = 0) -> bool:
        if 0 <= v1 <= self.n-1 and 0 <= v2 <= self.n-1:
            if v1 == v2:
                self.adj[v1].append(v2)
                self.adj[v1].sort()
                self.C[v1][v2] = weight
                return True
            self.adj[v1].append(v2)
            self.adj[v1].sort()
            self.C[v1][v2] = weight
            if not self.directed:
                self.adj[v2].append(v1)
                self.adj[v2].sort()
                self.C[v2][v1] = weight
            return True
        return False
            
    #Propiedades del grafo
    def degree(self, vertex) -> tuple:
        exit_deg = len(self.adj[vertex])
        if self.directed:
            if (vertex in self.adj[i] for i in range(self.n)):
                entry_deg += 1
            return entry_deg, exit_deg
        return None, exit_deg
    
    def max_degree(self) -> tuple:
        if self.directed:
            return max(self.deg_sequence()[0]), max(self.deg_sequence()[1])
        return None, max(self.deg_sequence())
    
    def min_degree(self) -> tuple:
        if self.directed:
            return min(self.deg_sequence()[0]), min(self.deg_sequence()[1])
        return None, min(self.deg_sequence())
    
    def deg_sequence(self) -> tuple:
        exit_seq = [self.degree(i)[1] for i in range(self.n)]
        if self.directed:
            entry_seq = [self.degree(i)[0] for i in range(self.n)]
            return entry_seq, exit_seq
        return exit_seq
    
    def num_of_components(self) -> int:
        components = 1
        for em in range(self.n):
            if (not self.path(em,i)[0] for i in range(self.n)):
                components += 1
        return components
        
    def is_connected(self):
        return True if self.num_of_components() == 1 else False
    
    def is_eulerian(self):
        pass
    
    def is_semieulerian(self):
        pass
    
    def is_r_regular(self, r: int):
        state = False
        for em in range(self.n - 1):
            if (self.degree(em) == r ):
                state = True
            else:
                state = False
        return state
    
    def is_complete(self):
        return True if self.is_r_regular(self.n - 1) else False
    
    def is_acyclic(self):
        for i in range(self.n):
            if (i in self.adj[i]):
                return False
            else:
                state = True
        return state
    
    #Recorridos
    def bfs(self, vertex):
        queue = []
        visited = [False] * self.n
        queue.append(vertex)
        visited[vertex] = True
        while len(queue) > 0:
            pointer = queue.pop(0)
            print(pointer, end = " ")
            for em in self.adj[pointer]:
                if not visited[em]:
                    queue.append(em)
                    visited[em] = True
        print()
        
    def _dfs(self, vertex, visited = None):
        visited[vertex] = True
        print(vertex , end = " ")
        for em in self.adj[vertex]:
            if not visited[em]:
                visited = self._dfs(em, visited)
        return visited
    
    def dfs(self, vertex):
        visited = [False]*self.n
        self._dfs(vertex, visited)
        print()
       
    #Herramientas
    def path(self, v1, v2) -> tuple:
        queue = []
        path = []
        visited = [False] * self.n
        queue.append(v1)
        visited[v1] = True
        while len(queue) > 0:
            pointer = queue.pop(0)
            path.append(pointer)
            if pointer == v2:
                if v2 > v1:
                    path.sort(reverse=True)
                    return True, path
                else:
                    path.sort()
                    return True, path
            for em in self.adj[pointer]:
                if not visited[em]:
                    queue.append(em)
                    visited[em] = True
        return False, []
    
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
            for i in self.adj[v]:
                if (D[v] + self.C[v][i] < D[i] and not visit[i]):
                    D[i] = D[v] + self.C[v][i]
                    pad[i] = v
        return D, pad
        
if __name__ == '__main__':
    test = Graph(10)
    
    A = 0
    B = 1
    C = 2
    D = 3
    E = 4
    F = 5
    G = 6
    H = 7
    I = 8
    J = 9
    
    test.add_edge(A,B,3) #ab3
    test.add_edge(A,C,8) #ac8
    test.add_edge(A,D,5) #ad5
    
    test.add_edge(B,C,5) #bc5 
    test.add_edge(B,F,7) #bf7
    
    test.add_edge(C,F,5) #cf5
    test.add_edge(C,E,8) #ce8
    test.add_edge(C,D,2) #cd2
    
    test.add_edge(D,G,4) #dg4
    
    test.add_edge(F,E,5) #fe5
    test.add_edge(F,H,6) #fh6
    
    test.add_edge(E,H,1) #eh1 
    test.add_edge(E,I,3) #ei3
    test.add_edge(E,G,6) #eg6
    
    test.add_edge(G,I,4) #gi4
    
    test.add_edge(H,J,2) #hj2
    
    test.add_edge(I,J,6) #ij6
    
    print("=====Adyacentes=====")
    print(test.adj)
    print()
    print("=====Costos=====")
    print(test.C)
    print()
    print("=====Degrees=====")
    print(tuple(test.deg_sequence()))
    print()
    print("min")
    print(tuple(test.min_degree()))
    print()
    print("max")
    print(tuple(test.max_degree()))
    print()
    print("=====Propiedades=====")
    print("Aciclico")
    print(test.is_acyclic())
    print()
    print("Completo")
    print(test.is_complete())
    print()
    print("2_regular")
    print(test.is_r_regular(2))
    print()
    print("=====Caminos Simples (bfs)=====")
    print(tuple(test.path(I, A)))
    print("=====Dijkstra(0)=====")
    distances, parents = test.dijkstra(0)
    print("Distancias")
    print(distances)
    print()
    print("Padres")
    print(parents)