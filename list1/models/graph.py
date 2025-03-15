from typing import Dict, List, Optional
from datetime import time
from dataclasses import dataclass
from geopy import Point

from const import PENATLY


@dataclass(frozen=True)
class Edge:
    company: str
    line: str
    start_node: "Node"
    end_node: "Node"
    arrival_time: time
    departure_time: time

    cost: int  # diff depatrute_time - arrival_time in min.

    def __str__(self):
        return (
            f"{self.line[:3].ljust(6)}"
            f"{str(self.departure_time).ljust(12)}"
            f"{self.start_node.name[:26].ljust(30)}"
            f"{str(self.arrival_time).ljust(12)}"
            f"{self.end_node.name[:26].ljust(30)}"
        )


@dataclass
class Node:
    name: str  # street name
    coords: Point
    edges: List["Edge"]

    def append_edge(self, e: "Edge"):
        self.edges.append(e)

    def __hash__(self):
        return hash(self.name)

    def __eq__(self, other):
        if isinstance(other, Node):
            return self.name == other.name
        return False

    def __lt__(self, other):
        if isinstance(other, Node):
            return self.name < other.name
        raise NotImplemented


class Graph:
    def __init__(self):
        self.__merged_nodes: Dict[str, Node] = {}

    @property
    def nodes(self) -> Dict[str, Node]:
        return self.__merged_nodes

    def add_merged_node(self, n: Node) -> None:
        if n.name not in self.nodes.keys():
            self.__merged_nodes[n.name] = n

    def update_edges(self, e: Edge) -> None:
        s_node = self.nodes[e.start_node.name]
        e_node = self.nodes[e.end_node.name]

        new_edge = Edge(
            company=e.company,
            line=e.line,
            start_node=s_node,
            end_node=e_node,
            arrival_time=e.arrival_time,
            departure_time=e.departure_time,
            cost=e.cost,
        )

        s_node.append_edge(new_edge)

    def compute_cost(self, e: Edge, curr_time: time) -> int:
        result = to_minutes(e.arrival_time) - to_minutes(curr_time)

        if to_minutes(e.departure_time) - to_minutes(curr_time) < 0:
            result = to_minutes(e.departure_time) + (24 * 60 - to_minutes(curr_time))

        return result

    def line_change_cost(self, edge: Optional[Edge], next_edge: Edge) -> int:
        if edge is not None and edge.line != next_edge.line:
            return PENATLY
        return 0

    def available_edges_from(
        self, start_node: Node, current_time: time | str
    ) -> List[Edge]:
        curr: int = to_minutes(current_time)
        max_time: int = 15 + curr

        filtered_edges = []

        for e in start_node.edges:
            dep_time = to_minutes(e.departure_time)
            if dep_time >= curr and dep_time <= max_time:
                filtered_edges.append(e)

        return filtered_edges


def to_minutes(t: str | time) -> int:
    if isinstance(t, str):
        lst = t.split(":")
        hours = int(lst[0])
        minutes = int(lst[1])
        return hours * 60 + minutes

    elif isinstance(t, time):
        return 60 * t.hour + t.minute

    return 0
