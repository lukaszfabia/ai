from datetime import time
import heapq
from models.graph import Graph
from printer import print_road
from search import SearchEngine
from timer import check_time
from typing import Optional, Dict
from models.graph import Node, Edge


class AStar(SearchEngine):

    def __init__(self, g: Graph):
        super().__init__(g)

    @check_time
    @print_road
    def search(self, start_point, end_point, start_time):
        """Find the best path by **minimizing time** using the A* algorithm.

        Args:
            start_point (str): Start stop, e.g., "Wyszyńskiego".
            end_point (str): End stop, e.g., "PL. GRUNWALDZKI".
            start_time (time): Describes the time when you arrive at the bus stop.

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
            _, current_node = heapq.heappop(frontier)

            if current_node == end_node:
                break

            for next_edge in self.graph.merged_neighbour_edges_for_start_node(
                current_node, current_time
            ):
                how_many += 1
                new_cost = cost_so_far[current_node] + self.graph.compute_cost(
                    next_edge, current_time
                )

                if (
                    next_edge.end_node not in cost_so_far
                    or new_cost < cost_so_far[next_edge.end_node]
                ):
                    cost_so_far[next_edge.end_node] = new_cost
                    priority = self.cost_strategy(
                        new_cost=new_cost,
                        end_node=end_node,
                        next_end_node=next_edge.end_node,
                    )
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

    def cost_strategy(self, new_cost, **kwargs):
        return new_cost + self._geo_heuristic(
            a=kwargs["end_node"], b=kwargs["next_end_node"]
        )

    def __str__(self):
        return "A*"


class AStarMutated(SearchEngine):

    def __init__(self, g):
        super().__init__(g)

    def cost_strategy(self, new_cost, **kwargs):
        return new_cost + self._geo_heuristic(
            a=kwargs["end_node"], b=kwargs["next_end_node"]
        )

    def __str__(self):
        return "A*"

    @check_time
    @print_road
    def search(self, start_point, end_point, start_time):
        """Minimize amount of transfers

        Args:
            start_point (str): Start stop, e.g., "Wyszyńskiego".
            end_point (str): End stop, e.g., "PL. GRUNWALDZKI".
            start_time (time): Describes the time when you arrive at the bus stop.

        """
