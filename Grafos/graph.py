class Graph():
    def __init__(self, n: int , directed = False) -> None:
        self.n = n
        self.directed = directed
        self.adj: list[list[int]] = [[] for _ in range(n)]
        
    def add_edge(self, v1, v2) -> bool:
        if 0 <= v1 <= self.n-1 and 0 <= v2 <= self.n-1:
            self.adj[v1].append(v2)
            self.adj[v1].sort()
            if not self.directed:
                self.adj[v2].append(v1)
                self.adj[v2].sort()
            return True
        return False
            
    #Propiedades del grafo
    def degree(self, vertex) -> int:
        return len(self.adj[vertex])
    
    def max_degree(self) -> int:
        return max(len(em) for em in self.adj)
    
    def min_degree(self) -> int:
        return min(len(em) for em in self.adj)
    
    def deg_sequence(self) -> list:
        seq = [len(em) for em in self.adj]
        return seq.sort()
    
    def num_of_components(self) -> int:
        pass
    
    def is_connected(self):
        return True if self.num_of_components() == 1 else False
    
    def is_eulerian(self):
        pass
    
    def is_semieulerian(self):
        pass
    
    def is_r_regular(self, r: int):
        return True if (self.degree(em) == r for em in range(self.n - 1)) else False
    
    def is_complete(self):
        return True if self.is_r_regular(self.n - 1) else False
    
    def is_acyclic(self):
        return True if (not i in self.adj[i] for i in range(self.n - 1)) else False
    
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
    def simple_path(self, v1, v2):
        pass
        
        
if __name__ == '__main__':
    test = Graph(13)
    
    test.add_edge(0,1)
    test.add_edge(0,2)
    test.add_edge(0,3)
    test.add_edge(1,3)
    test.add_edge(1,4)
    test.add_edge(2,3)
    test.add_edge(2,5)
    test.add_edge(2,6)
    test.add_edge(3,5)
    test.add_edge(3,6)
    test.add_edge(3,7)
    test.add_edge(4,7)
    test.add_edge(4,8)
    test.add_edge(5,9)
    test.add_edge(6,9)
    test.add_edge(7,10)
    test.add_edge(8,10)
    test.add_edge(8,11)
    test.add_edge(9,10)
    test.add_edge(9,12)
    test.add_edge(10,12)
    test.add_edge(11,12)
    test.add_edge(11,11)
    
    
    print("=====Es aciclico?=====")
    print(test.is_acyclic())
    print("=====Lista de adyacencia=====")
    print(test.adj)
    print("=====DFS(0)=====")
    test.dfs(0)
    print("=====DFS(5)=====")
    test.dfs(0)
    print("=====BFS(0)=====")
    test.bfs(0)
    print("=====BFS(5)=====")
    test.bfs(5)