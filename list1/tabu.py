from datetime import time
import random
from typing import List, Optional
from models.graph import Graph
from timer import check_time
from a import AStarMinTime, AStarMinTransfers
from parser import add_minutes_to_time


class Tabu:
    def __init__(
        self,
        g: Graph,
        t: time,
        points: List[str],
        src: str,
        max_iter: int = 50,
        minimize_time: bool = True,
    ):
        """

        Args:
            g (Graph): graph
            max_iter (int): max iterations
            t (time): start time
            points (List[str]): points to visit
            src (str): start stop
            minimize_time (bool, optional): flag, If true then we minimize time else minimize transfers. Defaults to True.
        """
        self.graph = g
        self.max_iter = max_iter
        self.time = t
        self.src = src
        self.points = points
        self.min_time = minimize_time

    def _generate_init_solution(self):
        solution = [self.src] + self.points.copy()
        random.shuffle(solution)
        return solution

    def set_points(self, new_points: List[str]):
        self.points = new_points

    def _generate_neighbors(self, solution: List[str]) -> List[List[str]]:
        neighbors = []
        for i in range(1, len(solution)):
            for j in range(i + 1, len(solution)):
                neighbor = solution.copy()
                neighbor[i], neighbor[j] = neighbor[j], neighbor[i]
                neighbors.append(neighbor)
        return neighbors

    def _compute_cost(self, solution: List[str]):
        cost = 0
        current_time = self.time
        current_line = None

        for i in range(len(solution) - 1):
            start_node = self.graph.nodes[solution[i]]
            edges = self.graph.available_edges_from(start_node, current_time)

            if not edges:
                return float("inf")

            edge = edges[0]
            current_time = edge.arrival_time

            if self.min_time:
                cost += self.graph.compute_cost(edge, current_time)
            else:
                if current_line is not None and edge.line != current_line:
                    cost += 1
                current_line = edge.line

        return cost

    def go_on_a_trip(self):
        curr_time = self.time
        curr_stop = self.src

        engine = (
            AStarMinTime(self.graph) if self.min_time else AStarMinTransfers(self.graph)
        )

        for point in self.points:
            res = engine.search(
                start_point=curr_stop, end_point=point, start_time=curr_time
            )
            curr_time = add_minutes_to_time(curr_time, res[-1])
            curr_stop = point

        res = engine.search(
            start_point=curr_stop, end_point=self.src, start_time=curr_time
        )
        curr_time = add_minutes_to_time(curr_time, res[-1])

    def _tabu_search(
        self, tabu_list, tabu_length=None, aspiration=False, sample_size=None
    ):
        current_solution = self._generate_init_solution()
        best_solution = current_solution
        best_cost = self._compute_cost(best_solution)

        for _ in range(self.max_iter):
            neighbors = self._generate_neighbors(current_solution)

            # sampling
            if sample_size is not None:
                neighbors = random.sample(neighbors, min(sample_size, len(neighbors)))

            best_neighbor = None
            best_neighbor_cost = float("inf")

            for neighbor in neighbors:
                neighbor_cost = self._compute_cost(neighbor)
                if aspiration and (
                    neighbor not in tabu_list or neighbor_cost < best_cost
                ):
                    if neighbor_cost < best_neighbor_cost:
                        best_neighbor = neighbor
                        best_neighbor_cost = neighbor_cost
                elif neighbor not in tabu_list:
                    if neighbor_cost < best_neighbor_cost:
                        best_neighbor = neighbor
                        best_neighbor_cost = neighbor_cost

            if best_neighbor is not None:
                current_solution = best_neighbor
                tabu_list.append(best_neighbor)

                if tabu_length is not None and len(tabu_list) > tabu_length:
                    tabu_list.pop(0)

                if best_neighbor_cost < best_cost:
                    best_solution = best_neighbor
                    best_cost = best_neighbor_cost

        return best_solution

    @check_time
    def search(self):
        return self._tabu_search(tabu_list=[])

    @check_time
    def dynamic_search(self):
        return self._tabu_search(tabu_list=[], tabu_length=10)

    @check_time
    def aspiration_search(self):
        return self._tabu_search(tabu_list=[], aspiration=True)

    @check_time
    def sampling_search(self, sample: Optional[int] = 40):
        return self._tabu_search(tabu_list=[], sample_size=sample)
