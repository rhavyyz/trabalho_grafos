from dataclasses import dataclass

@dataclass
class Edge:
    next_vertex : int
    weight : float


@dataclass
class EntireEdge:
    v : int
    w : int
    weight : float


