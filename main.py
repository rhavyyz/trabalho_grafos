from lib.graph import Graph
from lib.digraph import Digraph
from lib.fileUtils import FileUtils
# from lib.digraph import Digraph

# Tasks :
# 1. Input graph from file 

def main():
    filename = 'USA-road-d.NY.gr'
    graph_params = FileUtils.data_graph_from_file(filename)
    graph = Graph(graph_params[0][0], graph_params[0][1], True)
    FileUtils.fill_graph_from_file(graph, filename)
    digraph = Digraph(graph_params[0][0], graph_params[0][1], True)
    FileUtils.fill_graph_from_file(digraph, filename)
    
    #testes(graph)
    runTaskForGraph(graph)
    runTaskForDigraph(digraph)


def runTaskForGraph(graph):
    print('#'*20)
    print("Para um grafo orientado:")
    # a) Minimum degree
    print("a) Minimum degree:", graph.mind())

    # b) Maximum degree
    print("b) Maximum degree:", graph.maxd())

    # c) Path with >= 10 edges
    path = graph.find_long_path(10)
    print("c) Long path with >= 10 edges:", path)

    # # d) Cycle with >= 5 edges
    cycle = graph.find_cycle(5)
    print("d) Cycle with >= 5 edges:", cycle)

    # e) Farthest vertex from vertex 129
    distances, _ = graph.dijkstra(129)
    farthest_vertex, max_distance = max(enumerate(distances), key=lambda x: x[1])
    print("e) Farthest vertex from 129:", farthest_vertex + 1, "Distance:", max_distance)

    print('#'*20)

def runTaskForDigraph(digraph):
    print('#'*20)
    print("Para um grafo não orientado:(Digrafo)")
    
    # a) Minimum degree
    print("a) Minimum degree:", digraph.mind())

    # b) Maximum degree
    print("b) Maximum degree:", digraph.maxd())

    # c) Path with >= 10 edges
    path = digraph.find_long_path(10)
    print("c) Long path with >= 10 edges:", path)

    # # d) Cycle with >= 5 edges
    cycle = digraph.find_cycle(5)
    print("d) Cycle with >= 5 edges:", cycle)

    # e) Farthest vertex from vertex 129
    distances, _ = digraph.dijkstra(129)
    farthest_vertex, max_distance = max(enumerate(distances), key=lambda x: x[1])
    print("e) Farthest vertex from 129:", farthest_vertex + 1, "Distance:", max_distance)
    print('#'*20)

def testes(graph):
    # 1.A
    test_should_return_number_of_vertices(graph)
    # 1.B
    test_should_return_number_of_arcs(graph)
    # 1.C
    test_should_return_number_of_neighbors(graph)
    # 1.D
    test_should_return_degree_of_node(graph)
    # 1.E
    test_should_return_weight_of_arc(graph)
    # 1.F
    test_should_return_min_degree_of_node(graph)
    # 1.G
    test_should_return_max_degree_of_node(graph)
    # 1.H
    test_shoud_run_bfs_on_graph_and_return_d_pi(graph)
    # 1.I
    test_should_run_dfs_on_graph_and_return_pi_v_ini_v_fim(graph)

def test_should_return_number_of_vertices(graph):
    if(graph.n != 264346):
        print("a) Test failed")
        print(graph.n)
        return
    print("a) Vertice test passed")
    
def test_should_return_number_of_arcs(graph):
    if(graph.m != 733846):
        print("b) Test failed")
        print(graph.m)
        return
    print("b) Arcs test passed")
    
def test_should_return_number_of_neighbors(graph):
    vizinhos = graph.viz(1)
    if(len(vizinhos) != 3):
        print("c) Count Test failed")
        print(len(vizinhos))
        return
    if(vizinhos != [(2, 803), (12, 842), (1363, 2428)]):
        print("c) Array Comp Test failed")
        print(vizinhos)
        return
    print("c) Neighbors test passed")

def test_should_return_degree_of_node(graph):
    if(graph.d(1) != 3):
        print("d) Test failed")
        print(graph.d(1))
        return
    print("d) Degree test passed")

def test_should_return_weight_of_arc(graph):
    if(graph.w(1, 2) != 803):
        print("e) Test failed")
        print(graph.w(1, 2))
        return
    print("e) Weight test passed")

def test_should_return_min_degree_of_node(graph):
    if(graph.mind() != 1):
        print("f) Test failed")
        print(graph.mind())
        return
    print("f) Min Degree test passed")
    
def test_should_return_max_degree_of_node(graph):
    if(graph.maxd() != 8):
        print("g) Test failed")
        print(graph.maxd())
        return
    print("g) Max Degree test passed")

def test_shoud_run_bfs_on_graph_and_return_d_pi(graph):
    (d, pi) = graph.bfs(1)
    if(len(d) != 264346):
        print("h) Test failed")
        print(len(d))
        return
    if(len(pi) != 264346):
        print("h) Test failed")
        print(len(pi))
        return
    print("h) BFS test passed")

def test_should_run_dfs_on_graph_and_return_pi_v_ini_v_fim(graph):
    (pi, v_ini, v_fim) = graph.dfs(1)
    if(len(pi) != 264346):
        print("i) len(pi) -  Test failed")
        print(len(pi))
        return
    if(len(v_ini) != 264346):
        print("i) len(v_ini) - Test failed")
        print(len(v_ini))
        return
    if(len(v_fim) != 264346):
        print("i) len(v_fim) - Test failed")
        print(len(v_fim))
        return
    if(v_fim[len(v_fim)-1] != 246920):
        print("i) v_fim[len(v_fim)-1] - Test failed")
        print(v_fim[len(v_fim)-1])
        return
    print("i) DFS test passed")

if __name__ == "__main__":
    main()