from abc import ABC, abstractmethod
import heapq
import math
from typing import Dict, Optional

from models.graph import Edge, Graph, Node
from datetime import time

from printer import print_road
from timer import check_time

from geopy.distance import geodesic


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

    @abstractmethod
    def search(self, start_point: str, end_point: str, start_time: time):
        pass

    @abstractmethod
    def cost_strategy(self, new_cost: int, **kwargs) -> int:
        pass

    @abstractmethod
    def __str__(self):
        pass
