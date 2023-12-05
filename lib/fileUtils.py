from lib.graph import Graph

class FileUtils:
    @staticmethod
    def data_graph_from_file(filename):
        try:
            with open(filename, 'r') as file:  
                for line in file:
                    parts = line.split()
                    if line.startswith('p'):
                        info = parts[1]
                        node_count = int(parts[2])
                        edges_count = int(parts[3])
                        return (node_count,edges_count), None
        except FileNotFoundError:
            return None, "Arquivo não encontrado"  # Retorna None e uma mensagem de erro
        except Exception as e:
            return None, str(e)  # Retorna None e a mensagem de erro específica
        return None, "Linha não encontrada"
    
    @staticmethod
    def fill_graph_from_file(graph, filename):
        try:
            with open(filename, 'r') as file:
                for line in file:
                    parts = line.split()
                    if line.startswith('a'):
                        source = int(parts[1])
                        dest = int(parts[2])
                        weight = int(parts[3])
                        graph.add_edge(source, dest, weight)
        except FileNotFoundError:
            return None, "Arquivo não encontrado" 
        except Exception as e:
            return None, str(e)
        return None