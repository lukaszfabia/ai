# from typing import List


# def generate_neighbors(solution: List[str]) -> List[List[str]]:
#     neighbors = []
#     n = len(solution)
#     for i in range(n):
#         neighbor = solution[:]
#         j = (i + 1) % n
#         # Zamiana miejscami elementów w sąsiedzie
#         neighbor[i], neighbor[j] = neighbor[j], neighbor[i]
#         neighbors.append(neighbor)
#     return neighbors


# def tabu_search(g, start_point, points_to_visit, current_time, max_iterations, if_time):
#     time = current_time
#     curr_point = start_point
#     best_route = points_to_visit.copy()
#     new_route = points_to_visit.copy()
#     best_time = compute_route_time(
#         g, start_point, points_to_visit, current_time, if_time
#     )
#     tabu_list = set()
#     iteration = 0

#     while iteration < max_iterations:
#         candidates = generate_neighbors(new_route)

#         best_candidate = None
#         for candidate in candidates:
#             candidate_str = "-".join(candidate)
#             if candidate_str not in tabu_list:
#                 candidate_time = compute_route_time(
#                     g, start_point, candidate, time, if_time
#                 )
#                 if best_candidate is None or candidate_time < best_time:
#                     best_candidate = candidate
#                     best_time = candidate_time

#         if best_candidate is None:
#             tabu_list.clear()
#             continue

#         # Dodajemy do tabu listę
#         tabu_list.add("-".join(best_candidate))

#         new_route = best_candidate

#         iteration += 1

#     print(f"Best time Tabu {best_time}")
#     return best_route
