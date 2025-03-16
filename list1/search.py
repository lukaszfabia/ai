from abc import ABC, abstractmethod
import heapq
import math

from models.graph import Edge, Graph, Node
from datetime import time
from typing import Dict, Optional
from geopy.distance import geodesic

from printer import print_road
from timer import check_time


class SearchEngine(ABC):

    def __init__(self, g: Graph):
        self.graph = g

    def _manhattan_heuristic(self, a: Node, b: Node) -> int:
        return int(
            (
                abs(a.coords.latitude - b.coords.latitude)
                + abs(a.coords.longitude - b.coords.longitude)
            )
            * 1000
        )

    def _euklides_heuristic(self, a: Node, b: Node) -> int:
        return int(
            math.sqrt(
                (
                    math.pow(a.coords.latitude - b.coords.latitude, 2)
                    + math.pow(a.coords.longitude - b.coords.longitude, 2)
                )
            )
            * 1000
        )

    def _geo_heuristic(self, a: Node, b: Node) -> int:
        coords_a = (a.coords.latitude, a.coords.longitude)
        coords_b = (b.coords.latitude, b.coords.longitude)

        distance = geodesic(coords_a, coords_b).meters

        return int(distance)

    def is_worth(self, end_node: Node, cost_so_far, new_cost) -> bool:
        return end_node not in cost_so_far or new_cost < cost_so_far[end_node]

    @print_road
    @check_time
    def search(self, start_point: str, end_point: str, start_time: time):
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

            for next_edge in self.graph.available_edges_from(
                current_node, current_time
            ):
                how_many += 1

                new_cost = cost_so_far[current_node] + self.graph.compute_cost(
                    next_edge, current_time
                )

                if self.is_worth(next_edge.end_node, cost_so_far, new_cost):
                    cost_so_far[next_edge.end_node] = new_cost

                    priority = self.choose_path_strategy(
                        cost_so_far=cost_so_far,
                        current_node=current_node,
                        next_edge=next_edge,
                        current_time=current_time,
                        came_from=came_from,
                        end_node=end_node,
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

    @abstractmethod
    def cost_strategy(self, new_cost: int, **kwargs) -> int:
        pass

    @abstractmethod
    def choose_path_strategy(
        self,
        cost_so_far: Dict[Node, int],
        current_node: Node,
        next_edge: Edge,
        current_time: time,
        came_from: Dict[str, Optional[Edge]],
        end_node: Node,
    ) -> Optional[int]:
        pass

    def __str__(self):
        return __class__.__name__.__str__()
