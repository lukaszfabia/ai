from datetime import time
from typing import Dict, Optional
from models.graph import Edge, Graph, Node
from search import SearchEngine


class Dijkstra(SearchEngine):

    def __init__(self, g: Graph):
        super().__init__(g)

    def cost_strategy(self, new_cost, **kwargs):
        return new_cost

    def __str__(self):
        return self.__class__.__name__.__str__()

    def choose_path_strategy(
        self,
        cost_so_far: Dict[Node, int],
        current_node: Node,
        next_edge: Edge,
        current_time: time,
        came_from: Dict[str, Optional[Edge]],
        end_node: Node,
    ):
        new_cost = cost_so_far[current_node] + self.graph.compute_cost(
            next_edge, current_time
        )

        if self.is_worth(next_edge.end_node, cost_so_far, new_cost):
            cost_so_far[next_edge.end_node] = new_cost
            priority = self.cost_strategy(new_cost=new_cost)
            return priority
