from datetime import time
import heapq
from typing import Dict, Optional
from const import PENATLY
from models.graph import Edge, Graph, Node
from printer import print_road
from search import SearchEngine
from geopy.distance import geodesic
import numpy as np

from timer import check_time


def compute_angle(current: Node, next_node: Node, goal):
    v1 = np.array(
        [
            next_node.coords.longitude - current.coords.longitude,
            next_node.coords.latitude - current.coords.latitude,
        ]
    )
    v2 = np.array(
        [
            goal.coords.longitude - current.coords.longitude,
            goal.coords.latitude - current.coords.latitude,
        ]
    )

    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)

    if norm_v1 == 0 or norm_v2 == 0:
        return 0

    cos_theta = np.dot(v1, v2) / (norm_v1 * norm_v2)
    angle = np.degrees(np.arccos(np.clip(cos_theta, -1.0, 1.0)))

    return angle


def compute_angle_penalty(angle):
    if angle < 45:
        return 0
    elif angle < 90:
        return 100
    elif angle < 135:
        return 300
    else:
        return 500


class AStarMinTime(SearchEngine):

    def __init__(self, g: Graph):
        super().__init__(g)

    def choose_path_strategy(
        self,
        cost_so_far,
        current_node,
        next_edge,
        current_time,
        came_from,
        end_node,
    ):
        new_cost = cost_so_far[current_node] + self.graph.compute_cost(
            next_edge, current_time
        )

        cost_so_far[next_edge.end_node] = new_cost
        priority = self.cost_strategy(
            new_cost=new_cost,
            end_node=end_node,
            next_end_node=next_edge.end_node,
        )
        return priority

    def cost_strategy(self, new_cost, **kwargs):
        return new_cost + self._geo_heuristic(
            a=kwargs["end_node"], b=kwargs["next_end_node"]
        )

    def __str__(self):
        return __class__.__name__.__str__()


class AStarMinTransfers(SearchEngine):

    def __init__(self, g):
        super().__init__(g)

    def __str__(self):
        return __class__.__name__.__str__()

    def cost_strategy(self, new_cost, **kwargs):
        return new_cost + self._geo_heuristic(
            a=kwargs["end_node"], b=kwargs["next_end_node"]
        )

    def choose_path_strategy(
        self,
        cost_so_far,
        current_node,
        next_edge,
        current_time,
        came_from,
        end_node,
    ):
        new_cost = (
            cost_so_far[current_node]
            + self.graph.compute_cost(next_edge, current_time)
            + self.graph.line_change_cost(
                edge=came_from[current_node.name], next_edge=next_edge
            )
        )

        cost_so_far[next_edge.end_node] = new_cost
        priority = self.cost_strategy(
            new_cost=new_cost,
            end_node=end_node,
            next_end_node=next_edge.end_node,
        )
        return priority


class AStarModified(SearchEngine):
    def __init__(self, g):
        super().__init__(g)
        self._heuristic_cache = {}
        self._angle_cache = {}
        self._nodes_count = len(g.nodes)

    def __str__(self):
        return __class__.__name__.__str__()

    def _geo_heuristic(self, a: Node, b: Node) -> int:
        cache_key = (a.name, b.name)
        if cache_key in self._heuristic_cache:
            return self._heuristic_cache[cache_key]

        distance = geodesic(
            (a.coords.latitude, a.coords.longitude),
            (b.coords.latitude, b.coords.longitude),
        ).meters
        self._heuristic_cache[cache_key] = int(distance * 1.05)
        return self._heuristic_cache[cache_key]

    def _compute_angle_penalty(self, current_node, next_node, end_node):
        angle_key = (current_node.name, next_node.name, end_node.name)
        if angle_key not in self._angle_cache:
            angle = compute_angle(current_node, next_node, end_node)
            penalty = compute_angle_penalty(angle)
            self._angle_cache[angle_key] = penalty
        return self._angle_cache[angle_key]

    def cost_strategy(self, new_cost, **kwargs):
        progress = kwargs["cost_so_far"][kwargs["current_node"]] / max(new_cost, 1)
        angle_weight = max(0.5, 1.0 - progress * 0.8)
        angle_penalty = (
            self._compute_angle_penalty(
                kwargs["current_node"], kwargs["next_end_node"], kwargs["end_node"]
            )
            * angle_weight
        )

        heuristic = self._geo_heuristic(kwargs["end_node"], kwargs["next_end_node"])
        cost_weight = 0.7 if progress < 0.5 else 0.9
        heuristic_weight = 1.0 - cost_weight

        return int(
            new_cost * cost_weight + heuristic * heuristic_weight + angle_penalty
        )

    def choose_path_strategy(
        self, cost_so_far, current_node, next_edge, current_time, came_from, end_node
    ):
        new_cost = (
            cost_so_far[current_node]
            + self.graph.compute_cost(next_edge, current_time)
            + self.graph.line_change_cost(
                edge=came_from.get(current_node.name), next_edge=next_edge
            )
        )

        exploration_factor = len(came_from) / (self._nodes_count + 1)

        if exploration_factor > 0.2:
            heuristic = self._geo_heuristic(end_node, next_edge.end_node)
            progress = cost_so_far[current_node] / max(new_cost, 1)
            heuristic_weight = 0.5 + progress * 0.4
            priority = new_cost * (1 - heuristic_weight) + heuristic * heuristic_weight
        else:
            priority = self.cost_strategy(
                new_cost=new_cost,
                end_node=end_node,
                next_end_node=next_edge.end_node,
                current_node=current_node,
                cost_so_far=cost_so_far,
            )

        return priority

    @print_road
    @check_time
    def search_optimized(
        self, start_point: str, end_point: str, start_time: time, max_nodes=100000
    ):
        start_node = self.graph.nodes[start_point]
        end_node = self.graph.nodes[end_point]

        frontier = []
        heapq.heappush(frontier, (0, start_node))
        came_from = {}
        cost_so_far = {}
        came_from[start_node.name] = None
        cost_so_far[start_node] = 0

        how_many = 0
        current_time = start_time

        while frontier and how_many < max_nodes:
            _, current_node = heapq.heappop(frontier)

            if current_node == end_node:
                break

            edges = sorted(
                self.graph.available_edges_from(current_node, current_time),
                key=lambda e: self._geo_heuristic(e.end_node, end_node),
            )

            top_edges = int(len(edges) * 0.5)

            for next_edge in edges[:top_edges]:
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
