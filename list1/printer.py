from typing import List
from functools import wraps

from models.graph import Edge, to_minutes
from parser import minutes_to_hms


def print_road(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        end_node, came_from, cost_so_far, how_many, current_time, start_time, algo = (
            func(*args, **kwargs)
        )

        final_list: List[Edge] = []
        print("\n" + "=" * 100)
        print(f"Results for {algo}:")
        print("=" * 100)

        print(
            "Line".ljust(6)
            + "Departure".ljust(12)
            + "Start Stop".ljust(30)
            + "Arrival".ljust(12)
            + "End Stop".ljust(30)
            + "Cost".ljust(10)
        )
        print("-" * 100)

        current = end_node
        while current.name in came_from and came_from[current.name]:
            edge = came_from[current.name]
            final_list.append(edge)
            current = edge.start_node

        final_list.reverse()

        for edge in final_list:
            print(edge, f"{str(cost_so_far[edge.end_node]).ljust(10)}")
        total_time = to_minutes(current_time) - to_minutes(start_time)
        print("-" * 100)
        print(f"\nTotal time ({algo}): {minutes_to_hms(total_time)}")
        print(f"Total visited nodes: {how_many}")
        print("=" * 100 + "\n")

        return (
            end_node,
            came_from,
            cost_so_far,
            how_many,
            current_time,
            start_time,
            total_time,
        )

    return wrapper
