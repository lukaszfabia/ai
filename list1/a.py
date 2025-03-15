from models.graph import Graph, Node
from search import SearchEngine

import numpy as np


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

        if self.is_worth(next_edge.end_node, cost_so_far, new_cost):
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

        if self.is_worth(next_edge.end_node, cost_so_far, new_cost):
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

    def __str__(self):
        return __class__.__name__.__str__()

    def cost_strategy(self, new_cost, **kwargs):
        angle = compute_angle(
            current=kwargs["current_node"],
            next_node=kwargs["next_end_node"],
            goal=kwargs["end_node"],
        )
        angle_penalty = compute_angle_penalty(angle)
        priority = (
            new_cost
            + self._geo_heuristic(a=kwargs["end_node"], b=kwargs["next_end_node"])
            + angle_penalty
        )
        return priority

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

        if self.is_worth(next_edge.end_node, cost_so_far, new_cost):
            cost_so_far[next_edge.end_node] = new_cost
            priority = self.cost_strategy(
                new_cost=new_cost,
                end_node=end_node,
                next_end_node=next_edge.end_node,
                current_node=current_node,
            )
            return priority
