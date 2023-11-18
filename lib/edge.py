from dataclasses import dataclass

@dataclass
class Edge:
    next_vertex : int
    weight : float
