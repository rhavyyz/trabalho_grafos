from queue import Queue
import heapq
class Graph:
    def __init__(self,n,m, weighted=True):
        """
        Inicializa o dígrafo com um número especificado de vértices e arestas.

        Args:
            n (int): Número de vértices no dígrafo.
            m (int): Número de arestas no dígrafo.
            weighted (bool): Indica se o dígrafo é ponderado. Padrão é True.
        """
        self.n = n
        self.m = m
        self.weighted = weighted
        self.adjacency_list = [[] for _ in range(n)]

    def add_edge(self, source, to, weight=1):
        """
        Adiciona uma aresta ao dígrafo.

        Args:
            source (int): Vértice de origem da aresta.
            destination (int): Vértice de destino da aresta.
            weight (int, optional): Peso da aresta. Padrão é 1 se não ponderado.
        
        Regra: 
        * se o grafo for ponderado, não pode ter arestas paralelas
        * se ja existe uma aresta entre source e to, não adiciona
        """
        
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
        """
        Retorna os vizinhos do vértice especificado.

        Args:
            source (int): Vértice de origem.

        Returns:
            list: Lista de vértices vizinhos do vértice especificado.
        """
        return self.adjacency_list[source - 1]
    
    def d(self, source):
        """
        Retorna o grau de saída do vértice especificado.

        Args:
            source (int): Vértice de origem.

        Returns:
            int: Grau de saída do vértice.
        """
        return len(self.adjacency_list[source - 1])
    
    def maxd(self):
        """
        Retorna o grau de saída máximo no grafo.

        Returns:
            int: Grau de saída máximo.
        """
        return max(len(neighbors) for neighbors in self.adjacency_list)

    def mind(self):
        """
        Retorna o grau de saída mínimo no grafo.

        Returns:
            int: Grau de saída mínimo.
        """
        return min(len(neighbors) for neighbors in self.adjacency_list)

    
    def w(self, source, to):
        """
        Retorna o peso da aresta entre dois vértices especificados.

        Args:
            source (int): Vértice de origem.
            destination (int): Vértice de destino.

        Returns:
            int: Peso da aresta. Retorna 0 se a aresta não existir.
        """
        for neighbor in self.adjacency_list[source - 1]:
            if isinstance(neighbor, tuple) and neighbor[0] == to:
                return neighbor[1]
        return 0
    
    def bfs(self, source):
        """
        Realiza uma busca em largura (Breadth-First Search) a partir de um vértice fonte.

        Args:
            source (int): Vértice fonte para iniciar a busca.

        Returns:
            tuple: Retorna dois arrays, um com as distâncias do vértice fonte até cada vértice,
            e outro com os predecessores de cada vértice no caminho da busca.
        """
        
        visited = [False] * self.n # inicializa todos os vértices como não visitados
        d = [float('inf')] * self.n # inicializa todas as distâncias como infinito
        pi = [-1] * self.n # inicializa todos os predecessores como -1

        queue = Queue()
        queue.put(source)
        visited[source - 1] = True
        d[source - 1] = 0  
        pi[source - 1] = source 

        while not queue.empty():
            current = queue.get()
            for neighbor, weight in self.adjacency_list[current - 1]: # percorre os vizinhos do vértice atual
                if not visited[neighbor - 1]: # se o vizinho não foi visitado
                    visited[neighbor - 1] = True
                    d[neighbor - 1] = d[current - 1] + 1 
                    pi[neighbor - 1] = current 
                    queue.put(neighbor)

        return d, pi
    
    def dfs(self, source):
        """
        Realiza uma busca em profundidade (Depth-First Search) a partir de um vértice fonte.

        Args:
            source (int): Vértice fonte para iniciar a busca.

        Returns:
            tuple: Retorna três arrays, um com os predecessores de cada vértice no caminho da busca,
                um com os tempos de descoberta, e outro com os tempos de término.
        """
        visited = [False] * self.n # inicializa todos os vértices como não visitados
        pi = [-1] * self.n # inicializa todos os predecessores como -1
        v_ini = [-1] * self.n # inicializa todos os tempos de descoberta como -1
        v_end = [-1] * self.n # inicializa todos os tempos de término como -1

        stack = [(source, 'entry')]  # Entry indica que o vértice foi visitado mas seus vizinhos ainda não
        time = 0 # Inicializa o tempo

        while stack:
            vertex, action = stack.pop()

            if action == 'entry': # Se o vértice foi visitado mas seus vizinhos ainda não
                if visited[vertex - 1]:
                    continue

                visited[vertex - 1] = True
                time += 1
                v_ini[vertex - 1] = time
                stack.append((vertex, 'exit'))  # Adiciona o vértice na pilha para ser visitado novamente quando seus vizinhos forem visitados

                for neighbor, _ in self.adjacency_list[vertex - 1]: # Percorre os vizinhos do vértice atual
                    if not visited[neighbor - 1]:
                        pi[neighbor - 1] = vertex
                        stack.append((neighbor, 'entry')) # Adiciona os vizinhos do vértice atual na pilha para serem visitados

            elif action == 'exit': # Se o vértice foi visitado e seus vizinhos também
                time += 1 # Incrementa o tempo
                v_end[vertex - 1] = time # Armazena o tempo de término do vértice

        return pi, v_ini, v_end
    
    def bellman_ford(self, source):
        """
        Executa o algoritmo Bellman-Ford para encontrar as distâncias mais curtas a partir de um vértice fonte em um dígrafo.
        Esse algoritmo também detecta ciclos de peso negativo.

        Args:
            source (int): Vértice fonte para iniciar o algoritmo.

        Returns:
            tuple: Retorna dois arrays, um com as distâncias do vértice fonte até cada vértice,
                e outro com os predecessores de cada vértice no caminho.
                Lança um ValueError se um ciclo de peso negativo for encontrado.
        """
        d = [float('inf')] * self.n # inicializa todas as distâncias como infinito
        pi = [-1] * self.n # inicializa todos os predecessores como -1
        d[source - 1] = 0  # distância do vértice fonte para ele mesmo é 0

        # Relaxa todas as arestas n - 1 vezes
        for _ in range(self.n - 1):
            for u in range(self.n):
                for v, w in self.adjacency_list[u]:
                    if d[u] + w < d[v - 1]:
                        d[v - 1] = d[u] + w
                        pi[v - 1] = u + 1

        # Verifica se há ciclos de peso negativo
        for u in range(self.n):
            for v, w in self.adjacency_list[u]:
                if d[u] + w < d[v - 1]:
                    raise ValueError("Graph contem um ciclo de peso negativo")

        return d, pi

    def dijkstra(self, source):
        """
        Executa o algoritmo de Dijkstra para encontrar as distâncias mais curtas a partir de um vértice fonte em um dígrafo.

        Args:
            source (int): Vértice fonte para iniciar o algoritmo.

        Returns:
            tuple: Retorna dois arrays, um com as distâncias do vértice fonte até cada vértice,
                e outro com os predecessores de cada vértice no caminho.
        """
        d = [float('inf')] * self.n # inicializa todas as distâncias como infinito
        pi = [-1] * self.n # inicializa todos os predecessores como -1
        d[source - 1] = 0 # distância do vértice fonte para ele mesmo é 0
        queue = [(0, source)] # inicializa a fila de prioridade com o vértice fonte
        
        # Enquanto a fila de prioridade não estiver vazia
        while queue:
            # Remove o vértice com menor distância da fila de prioridade
            dist, u = heapq.heappop(queue) 
            if dist > d[u - 1]:
                continue 
            # Relaxa todas as arestas do vértice removido
            for v, weight in self.adjacency_list[u - 1]:
                if d[u - 1] + weight < d[v - 1]:
                    d[v - 1] = d[u - 1] + weight
                    pi[v - 1] = u
                    # Adiciona o vértice na fila de prioridade com a nova distância
                    heapq.heappush(queue, (d[v - 1], v))
                    
        return d, pi

    def find_long_path(self, min_length):
        """
        Encontra um caminho no grafo com um número de arestas maior ou igual a min_length.

        Args:
            min_length (int): O número mínimo de arestas que o caminho deve ter.
        Returns:
            list: Um caminho no grafo que atende ao requisito de comprimento, ou None se nenhum for encontrado.
        """
        # Tenta encontrar um caminho com o comprimento mínimo a partir de cada vértice
        for start_vertex in range(1, self.n + 1):
            # Se um caminho for encontrado, retorna ele
            path = self.dfs_path(start_vertex, min_length)
            if path is not None:
                return path
        return None
    
    def dfs_path(self, start_vertex, min_length):
        """
        Realiza uma busca em profundidade a partir de um vértice de início, procurando um caminho com o comprimento mínimo especificado.

        Args:
            start_vertex (int): Vértice de início para a busca.
            min_length (int): Comprimento mínimo do caminho.

        Returns:
            list: Um caminho que atende ao comprimento mínimo, ou None se nenhum for encontrado.
        """
        visited = [False] * self.n # inicializa todos os vértices como não visitados
        stack = [(start_vertex, [start_vertex])] # inicializa a pilha com o vértice de início

        # Enquanto a pilha não estiver vazia
        while stack:
            # Remove o vértice do topo da pilha
            vertex, path = stack.pop()
            if visited[vertex - 1]:
                continue

            visited[vertex - 1] = True
            # Se o caminho atual atende ao requisito de comprimento, retorna ele
            for neighbor, _ in self.adjacency_list[vertex - 1]:
                if not visited[neighbor - 1]:
                    # Adiciona o vizinho na pilha com o caminho atualizado
                    new_path = path + [neighbor]
                    if len(new_path) - 1 >= min_length:  # -1 pois estamos contando arestas, não vértices
                        return new_path
                    # Adiciona o vizinho na pilha com o caminho atualizado
                    stack.append((neighbor, new_path))
        return None
    
    def find_cycle(self, min_length):
        """
        Tenta encontrar um ciclo no dígrafo com um comprimento mínimo especificado.

        Args:
            min_length (int): Comprimento mínimo do ciclo.

        Returns:
            list: Um ciclo que atende ao comprimento mínimo, ou None se nenhum for encontrado.
        """
        # Tenta encontrar um ciclo com o comprimento mínimo a partir de cada vértice
        for vertex in range(1, self.n + 1):
            # Se um ciclo for encontrado, retorna ele
            if cycle := self.dfs_find_cycle(vertex, min_length):
                return cycle
        return None

    def dfs_find_cycle(self, start_vertex, min_length):
        """
        Realiza uma busca em profundidade para encontrar um ciclo de um comprimento mínimo especificado.

        Args:
            start_vertex (int): Vértice de início para a busca.
            min_length (int): Comprimento mínimo do ciclo.

        Returns:
            list: Um ciclo que atende ao comprimento mínimo, ou None se nenhum for encontrado.
        """
        visited = [False] * self.n # inicializa todos os vértices como não visitados
        parent = [-1] * self.n # inicializa todos os predecessores como -1
        stack = [(start_vertex, 0)] # inicializa a pilha com o vértice de início

        while stack:
            # Remove o vértice do topo da pilha
            vertex, depth = stack.pop()
            if visited[vertex - 1]:
                continue

            visited[vertex - 1] = True
            # Se o ciclo atual atende ao requisito de comprimento, retorna ele
            for neighbor, _ in self.adjacency_list[vertex - 1]:
                if visited[neighbor - 1] and parent[vertex - 1] != neighbor:
                    if depth >= min_length - 1:  # Certifica-se de que o ciclo tem o tamanho mínimo
                        cycle = self.reconstruct_cycle(parent, vertex, neighbor)
                        if len(cycle) >= min_length + 1:  # +1 porque estamos contando vértices, não arestas
                            return cycle
                if not visited[neighbor - 1]:
                    parent[neighbor - 1] = vertex
                    # Adiciona o vizinho na pilha com a profundidade atualizada
                    stack.append((neighbor, depth + 1))
        return None
    
    def reconstruct_cycle(self,parent, start, end):
        """
        Reconstrói o ciclo a partir de informações de predecessores.

        Args:
            parent (list): Lista de predecessores de cada vértice.
            start (int): Vértice inicial do ciclo.
            end (int): Vértice final do ciclo.

        Returns:
            list: Um ciclo reconstruído.
        """
        # Reconstrói o ciclo a partir do vetor de predecessores
        cycle = [end] # inicializa o ciclo com o vértice final
        # Continua adicionando vértices até chegar no vértice inicial
        while start != end:
            # Adiciona o vértice no ciclo e atualiza o vértice atual
            cycle.append(start)
            # Atualiza o vértice atual para o seu predecessor
            start = parent[start - 1]
        # Adiciona o vértice inicial no ciclo e retorna ele
        cycle.append(end)
        # Inverte o ciclo para que ele fique na ordem correta
        return cycle[::-1]