from lib.edge import Edge
from queue import Queue, PriorityQueue

class Graph:

    __digraph = False


    ############################# Exception tests #########################################
    def ___test_for_out_of_range_exception(self, *args):
        for arg in args:
            if arg >= len(self.__g):
                raise Exception(f"Vertex out of range: [0, {len(self.__g) -1}]")             
            
    def ___test_for_self_edge_not_permited_exception(self, v ,w):
        if v == w:
            raise Exception("Self edge not permited")
    #######################################################################################

    def __init__(self, n) -> None:
        if n < 0:
            raise Exception('A graph should contain at least 0 vertices')

        self.__g : list[list[Edge]] = []
        self.__g += [[]]  * n


    def add_edge(self, v : int, w : int, weigth : float = 1):
        self.___test_for_out_of_range_exception(v,w)
        self.___test_for_self_edge_not_permited_exception(v, w)

        for edge in self.__g[v]:
            if edge.next_vertex == w:
                raise Exception(f'Cant register 2 edges from {v} to {w}')

        self.__g[v].append(Edge(w, weigth))
        
        if not self.__digraph:    
            self.__g[w].append(Edge(v, weigth))

    def remove_edge(self, v : int, w : int):
        self.___test_for_out_of_range_exception(v,w)

        weigth = None

        for edge in self.__g[v]:
            if edge.next_vertex == w:
                weigth = edge.weight
                break

        if weigth == None:
            return
        
        self.__g[v].remove(Edge(w, weight))

        if not self.__digraph:    
            self.__g[w].remove(Edge(v, weigth=weigth))


        return len(self.__g[v])


    ################################# Requiriments ########################################
    def n(self):
        return len(self.__g)
    
    def m(self):
        tot = 0

        for adjacency_list in self.__g:
            tot += len(adjacency_list)

        return tot / 2
    
    def d(self, v : int):
        self.___test_for_out_of_range_exception(v)

        return len(self.__g[v])
    
    def viz(self, v : int):
        self.___test_for_out_of_range_exception(v)

        return self.__g[v].copy()
    
    def mind(self):
        if len(self.__g) == 0:
            return None
        
        min = len(self.__g[0])

        for c in range(1, len(self.__g)):
            if min > len(self.__g[c]):
                min = len(self.__g[c])

        return min
    
    def maxd(self):
        if len(self.__g) == 0:
            return None
        
        max = len(self.__g[0])

        for c in range(1, len(self.__g)):
            if max < len(self.__g[c]):
                max = len(self.__g[c])

        return max

    def bfs(self, v : int):
        self.___test_for_out_of_range_exception(v)

        visited = [False] * len(self.__g)
        order = Queue(-1) 
        order.put((v, v ,0))

        d_list = [float('inf')] * len(self.__g)
        pi_list = [-1] * len(self.__g)

        while not order.empty() > 0:
            vertex, last, dist = order.get_nowait()
            
            if visited[vertex]: 
                continue

            visited[vertex] = True
            d_list[v] = dist
            pi_list[vertex] = last


            for next, next_dist in self.__g[vertex]:
                order.put((next, vertex, next_dist + dist))


        return (d_list, pi_list)


    def dfs(self, v : int):
        self.___test_for_out_of_range_exception(v)

        visited = [False] * len(self.__g)
        order = [(v,v,0)]

        pi_list = [-1] * len(self.__g)

        raise NotImplementedError()

        while not len(order) > 0:
            vertex, last, dist = order.pop()
            
            if visited[vertex]: 
                continue

            visited[vertex] = True
            d_list[v] = dist
            pi_list[vertex] = last


            for next, next_dist in self.__g[vertex]:
                order.append((next, vertex, next_dist + dist))


        return (d_list, pi_list)

    def bf(self, v: int):
        raise NotImplementedError()

    def djikstra(self, v : int):
        self.___test_for_out_of_range_exception(v)

        visited = [False] * len(self.__g)
        order = PriorityQueue(-1) 
        order.put((0,v, v))

        d_list = [float('inf')] * len(self.__g)
        pi_list = [-1] * len(self.__g)

        while not order.empty() > 0:
            dist, vertex, last = order.get_nowait()
            
            if visited[vertex]: 
                continue

            visited[vertex] = True
            d_list[v] = dist
            pi_list[vertex] = last


            for next, next_dist in self.__g[vertex]:
                order.put((next_dist + dist, next, vertex))

        return (d_list, pi_list)

        


    #######################################################################################

# if __name__ == "__main__":
#     main()