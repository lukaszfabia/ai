import heapq
from datetime import time
from typing import Dict, Optional
from models.graph import Edge, Graph, Node
from printer import print_road
from search import SearchEngine
from timer import check_time


class Dijkstra(SearchEngine):

    def __init__(self, g: Graph):
        super().__init__(g)

    def cost_strategy(self, new_cost, **kwargs):
        return new_cost

    def __str__(self):
        return "Dijkstra"

    @check_time
    @print_road
    def search(self, start_point: str, end_point: str, start_time: time):
        """Find best path by **minimizing time**

        Args:
            start_point (str): start stop, eg. Wyszyńskiego
            end_point (str): end stop, PL. GRUNWALDZKI
            start_time (time): describes time when you arrive on the bus stop

        Returns:
            Answer you can see in the console or notebook
        """
        start_node = self.graph.nodes[start_point]
        end_node = self.graph.nodes[end_point]

        frontier = []
        heapq.heappush(frontier, (0, start_node))
        came_from: Dict[str, Optional[Edge]] = {}
        cost_so_far: Dict[Node, int] = {}
        came_from[start_node.name] = None
        cost_so_far[start_node] = 0

        how_many = 0
        current_time = start_time

        while frontier:
            current_cost, current_node = heapq.heappop(frontier)

            if current_node == end_node:
                break

            for next_edge in self.graph.merged_neighbour_edges_for_start_node(
                current_node, current_time
            ):
                how_many += 1
                new_cost = current_cost + self.graph.compute_cost(
                    next_edge, current_time
                )

                if (
                    next_edge.end_node not in cost_so_far
                    or new_cost < cost_so_far[next_edge.end_node]
                ):
                    cost_so_far[next_edge.end_node] = new_cost
                    priority = self.cost_strategy(new_cost=new_cost)
                    heapq.heappush(frontier, (priority, next_edge.end_node))
                    came_from[next_edge.end_node.name] = next_edge

            if frontier:
                next_node = frontier[0][1]
                if next_node.name in came_from and came_from[next_node.name]:
                    current_time = came_from[next_node.name].arrival_time

        return (
            end_node,
            came_from,
            cost_so_far,
            how_many,
            current_time,
            start_time,
            self.__str__(),
        )
