from queue import Queue
import heapq
class Graph:
    def __init__(self,n,m, weighted=True):
        self.n = n # Number of vertices
        self.m = m # Number of edges
        self.weighted = weighted # Whether the graph is weighted or not
        self.adjacency_list = [[] for _ in range(n)] # 

    def add_edge(self, source, to, weight=1):
        if self.weighted:
            if any(to == neighbor for neighbor, _ in self.adjacency_list[source - 1]):
                return
            self.adjacency_list[source - 1].append((to, weight))
            self.adjacency_list[to - 1].append((source, weight))
        else:
            if any(to == neighbor for neighbor, _ in self.adjacency_list[source - 1]):
                return
            self.adjacency_list[source - 1].append((to, 1))
            self.adjacency_list[to - 1].append((source, 1))

    def viz(self, source):
        return self.adjacency_list[source - 1]
    
    def d(self, source):
        return len(self.adjacency_list[source - 1])
    
    def maxd(self):
        return max(len(neighbors) for neighbors in self.adjacency_list)

    def mind(self):
        return min(len(neighbors) for neighbors in self.adjacency_list)

    
    def w(self, source, to):
        for neighbor in self.adjacency_list[source - 1]:
            if isinstance(neighbor, tuple) and neighbor[0] == to:
                return neighbor[1]
        return 0
    
    def bfs(self, source):
        visited = [False] * self.n
        d = [float('inf')] * self.n  # Distance from source to each node
        pi = [-1] * self.n  # Parent of each node in the path from the source

        queue = Queue()
        queue.put(source)
        visited[source - 1] = True
        d[source - 1] = 0  # Distance from source to itself is 0
        pi[source - 1] = source  # Parent of the source is itself

        while not queue.empty():
            current = queue.get()
            for neighbor, weight in self.adjacency_list[current - 1]:
                if not visited[neighbor - 1]:
                    visited[neighbor - 1] = True
                    d[neighbor - 1] = d[current - 1] + 1  # Increment the distance
                    pi[neighbor - 1] = current  # Set the parent
                    queue.put(neighbor)

        return d, pi
    
    def dfs(self, source):
        visited = [False] * self.n
        pi = [-1] * self.n
        v_ini = [-1] * self.n
        v_end = [-1] * self.n

        stack = [(source, 'entry')]  # Stack of (vertex, entry/exit)
        time = 0

        while stack:
            vertex, action = stack.pop()

            if action == 'entry':
                if visited[vertex - 1]:
                    continue

                visited[vertex - 1] = True
                time += 1
                v_ini[vertex - 1] = time
                stack.append((vertex, 'exit'))  # Add for finish time

                for neighbor, _ in self.adjacency_list[vertex - 1]:
                    if not visited[neighbor - 1]:
                        pi[neighbor - 1] = vertex
                        stack.append((neighbor, 'entry'))

            elif action == 'exit':
                time += 1
                v_end[vertex - 1] = time

        return pi, v_ini, v_end
    
    def bellman_ford(self, source):
        d = [float('inf')] * self.n  # Initialize distances to infinity
        pi = [-1] * self.n  # Initialize predecessors to -1 (no predecessor)

        d[source - 1] = 0  # Distance from source to itself is 0

        # Relax edges repeatedly
        for _ in range(self.n - 1):
            for u in range(self.n):
                for v, w in self.adjacency_list[u]:
                    if d[u] + w < d[v - 1]:
                        d[v - 1] = d[u] + w
                        pi[v - 1] = u + 1

        # Check for negative weight cycles
        for u in range(self.n):
            for v, w in self.adjacency_list[u]:
                if d[u] + w < d[v - 1]:
                    raise ValueError("Graph contains a negative-weight cycle")

        return d, pi

    def dijkstra(self, source):
            d = [float('inf')] * self.n  # Initialize distances to infinity
            pi = [-1] * self.n  # Initialize predecessors to -1 (no predecessor)

            d[source - 1] = 0  # Distance from source to itself is 0
            queue = [(0, source)]  # Priority queue, initialized with source

            while queue:
                dist, u = heapq.heappop(queue)  # Vertex with the shortest distance
                if dist > d[u - 1]:
                    continue  # Skip if we have already found a shorter path

                for v, weight in self.adjacency_list[u - 1]:
                    if d[u - 1] + weight < d[v - 1]:
                        d[v - 1] = d[u - 1] + weight
                        pi[v - 1] = u
                        heapq.heappush(queue, (d[v - 1], v))

            return d, pi

    def find_long_path(self, min_length):
        for start_vertex in range(1, self.n + 1):
            _, v_ini, v_end = self.dfs(start_vertex)
            path = [i for i, v in enumerate(v_ini) if v != -1]  # vertices visited
            if len(path) >= min_length:
                return path[:min_length]
        return None
        
    def find_cycle(self, min_length):
        for vertex in range(1, self.n + 1):
            if cycle := self.dfs_find_cycle(vertex, min_length):
                return cycle
        return None

    def dfs_find_cycle(self, start_vertex, min_length):
        visited = [False] * self.n
        parent = [-1] * self.n
        stack = [(start_vertex, 0)]

        while stack:
            vertex, depth = stack.pop()
            if visited[vertex - 1]:
                continue

            visited[vertex - 1] = True
            for neighbor, _ in self.adjacency_list[vertex - 1]:
                if visited[neighbor - 1] and parent[vertex - 1] != neighbor and depth >= min_length - 1:
                    return self.reconstruct_cycle(parent, vertex, neighbor)
                if not visited[neighbor - 1]:
                    parent[neighbor - 1] = vertex
                    stack.append((neighbor, depth + 1))

        return None

    def reconstruct_cycle(self,parent, start, end):
        cycle = [end]
        while start != end:
            cycle.append(start)
            start = parent[start - 1]
        cycle.append(end)
        return cycle[::-1]