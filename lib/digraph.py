from lib.graph import Graph


class Digraph(Graph):
    def __init__(self,n , *args, **kargs) -> None:
        
        self.__digraph = True
    
        super().__init__(n, *args, **kargs)