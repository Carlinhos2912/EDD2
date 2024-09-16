class Graph():
    def __init__(self, lista_adyacencia = None) -> None:
        self.lista_adyacencia = lista_adyacencia
        
    def bfs(self, vertex):
        queue = []
        visited = [False]*len(self.lista_adyacencia)
        queue.append(vertex)
        visited[vertex] = True
        while len(queue) > 0:
            pointer = queue.pop(0)
            print(pointer)
            for em in self.lista_adyacencia[pointer]:
                if not visited[em]:
                    queue.append(em)
                    visited[em] = True
        
    def dfs(self, vertex, visited = None):
        if not visited:
            visited = [False]*len(self.lista_adyacencia)
        
        visited[vertex] = True
        print(vertex)
        for em in self.lista_adyacencia[vertex]:
            if not visited[em]:
                visited = self.dfs(em, visited)
        return visited
            
    def insert(self, adjacents: list) -> None:
        self.lista_adyacencia
    
    def connect(self, v1, v2):
        try:
            (self.lista_adyacencia[v1] and self.lista_adyacencia[v2])
            print("Siuuuu")
        except:
            print("Dojoda")
                    
test = Graph()
test.lista_adyacencia = [[1, 3],[0, 2, 3, 5, 8],[1, 6, 8],[0, 1, 5, 6],[],[1, 3, 7],[2, 3],[5],[1, 2],[]]

print("=====DFS=====")
test.dfs(0)
print("=====BFS=====")
test.bfs(0)