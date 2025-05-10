import sys
import heapq
import numpy as np
from collections import defaultdict
from time import time


def read_data(file_path=None):
    """Read input data from file or stdin"""
    if file_path:
        with open(file_path, 'r') as f:
            lines = f.readlines()
    else:
        lines = sys.stdin.readlines()

    n, k = map(int, lines[0].split())
    distance_matrix = []

    for i in range(1, n + 2):
        row = list(map(int, lines[i].split()))
        distance_matrix.append(row)

    return n, k, distance_matrix

class State:
    def __init__(self, n, k):
        self.n = n
        self.k = k
        self.routes = [[] for _ in range(k)]
        self.distances = [0] * k
        self.unvisited = set(range(1, n + 1))
        self.lower_bound = 0
        self.depth = 0  # Track search depth

    def __lt__(self, other):
        # Best-first search with tie-breaking
        if abs(self.lower_bound - other.lower_bound) < 1e-6:
            return self.depth > other.depth  # Prefer deeper nodes when bounds are equal
        return self.lower_bound < other.lower_bound

    def clone(self):
        new_state = State(self.n, self.k)
        new_state.routes = [route[:] for route in self.routes]
        new_state.distances = self.distances.copy()
        new_state.unvisited = self.unvisited.copy()
        new_state.lower_bound = self.lower_bound
        new_state.depth = self.depth + 1
        return new_state


# Preprocessing: Calculate minimum outgoing distance for each node
def preprocess_distances(distances, n):
    min_outgoing = [float('inf')] * (n + 1)
    for i in range(n + 1):
        for j in range(n + 1):
            if i != j:
                min_outgoing[i] = min(min_outgoing[i], distances[i][j])
    return min_outgoing


# Greedy solution to establish initial upper bound
def greedy_solution(n, k, distances):
    state = State(n, k)
    for i in range(k):
        state.routes[i].append(0)  # Add depot to each route

    # Sort nodes by distance from depot
    nodes = sorted(range(1, n + 1), key=lambda x: distances[0][x])

    for node in nodes:
        # Find vehicle with minimum current distance
        vehicle_id = min(range(k), key=lambda v: state.distances[v])
        last_node = state.routes[vehicle_id][-1]

        # Add node to route
        state.routes[vehicle_id].append(node)
        state.distances[vehicle_id] += distances[last_node][node]



    return state, max(state.distances)


def compute_lower_bound(state, distances, min_outgoing):
    """Compute a tighter lower bound using multiple techniques"""
    # Current max distance
    current_max = max(state.distances) if state.distances else 0

    # If all nodes are visited, return the current max distance
    if not state.unvisited:
        return current_max

    # 1. Calculate minimum insertion cost for each unvisited node
    min_insertion_cost = float('inf')
    for node in state.unvisited:
        best_insertion = float('inf')
        for vehicle_id in range(state.k):
            last_node = state.routes[vehicle_id][-1]
            insertion_cost = distances[last_node][node]
            best_insertion = min(best_insertion, insertion_cost)
        min_insertion_cost = min(min_insertion_cost, best_insertion)

    # 2. Calculate minimum spanning tree (approximated with minimum outgoing edges)
    mst_cost = 0
    for node in state.unvisited:
        mst_cost += min_outgoing[node]

    # 3. Even distribution bound
    remaining_nodes = len(state.unvisited)
    min_vehicles_needed = remaining_nodes // state.k + (1 if remaining_nodes % state.k else 0)

    # Calculate final lower bound as maximum of different estimates
    return max(current_max,
               current_max + min_insertion_cost,
               min(state.distances) + mst_cost / state.k)


def min_max_vrp(n, k, distances):
    start_time = time()
    timeout = 15

    # Preprocessing
    min_outgoing = preprocess_distances(distances, n)

    # Get initial solution using greedy approach
    initial_solution, best_max_distance = greedy_solution(n, k, distances)
    best_solution = initial_solution

    print(f"Initial greedy solution: {best_max_distance}")

    # Initialize priority queue with root node
    root = State(n, k)
    for i in range(k):
        root.routes[i].append(0)  # Add depot to each route
    root.lower_bound = compute_lower_bound(root, distances, min_outgoing)

    queue = [root]
    nodes_explored = 0

    # Dominance relations: track best state for each set of unvisited nodes
    best_states = {}

    while queue and nodes_explored < 1000000 and time() - start_time < timeout:
        if nodes_explored % 10000 == 0:
            print(f"Nodes explored: {nodes_explored}, Queue size: {len(queue)}")

        nodes_explored += 1
        current = heapq.heappop(queue)

        # Skip if we already have a better state with the same unvisited set
        unvisited_key = frozenset(current.unvisited)
        if unvisited_key in best_states:
            max_dist = max(current.distances)
            if max_dist >= best_states[unvisited_key]:
                continue
            best_states[unvisited_key] = max_dist
        else:
            best_states[unvisited_key] = max(current.distances)

        # If all nodes have been visited
        if not current.unvisited:
            # Complete routes by returning to depot
            max_distance = 0
            max_distance = max(current.distances)

            if max_distance < best_max_distance:
                best_max_distance = max_distance
                best_solution = current
                print(f"New best solution found: {best_max_distance}")

            continue

        # Prune if lower bound exceeds best solution
        if current.lower_bound >= best_max_distance:
            continue

        # Variable ordering: select node with minimum insertion cost
        insertion_costs = []
        for node in current.unvisited:
            min_cost = float('inf')
            best_vehicle = -1
            for vehicle_id in range(k):
                last_node = current.routes[vehicle_id][-1]
                cost = distances[last_node][node]
                if cost < min_cost:
                    min_cost = cost
                    best_vehicle = vehicle_id
            insertion_costs.append((min_cost, node, best_vehicle))

        # Sort by insertion cost (focus on most promising nodes first)
        insertion_costs.sort()

        # Limit branching factor for large problems
        max_branching = 3 if len(current.unvisited) > 10 else len(current.unvisited)

        for _, node_id, best_vehicle in insertion_costs[:max_branching]:
            vehicles_to_try = [best_vehicle] + [v for v in range(k) if v != best_vehicle]

            if len(current.unvisited) > 5:
                vehicles_to_try = vehicles_to_try[:2]

            for vehicle_id in vehicles_to_try:
                new_state = current.clone()
                last_node = new_state.routes[vehicle_id][-1]

                new_state.unvisited.remove(node_id)
                new_state.routes[vehicle_id].append(node_id)
                new_state.distances[vehicle_id] += distances[last_node][node_id]

                if new_state.distances[vehicle_id] >= best_max_distance:
                    continue

                new_state.lower_bound = compute_lower_bound(new_state, distances, min_outgoing)

                if new_state.lower_bound < best_max_distance:
                    heapq.heappush(queue, new_state)

    print(f"Search completed. Explored {nodes_explored} nodes in {time() - start_time:.2f} seconds")
    return best_solution


def format_solution(solution):
    output = [str(solution.k)]
    for route in solution.routes:
        if route[-1] == 0:
            route = route[:-1]
        output.append(str(len(route)))
        output.append(' '.join(map(str, route)))
    return '\n'.join(output)

def calculate_vehicle_ranges(solution, distances):

    ranges = []

    for vehicle_id, route in enumerate(solution.routes):
        total_distance = 0

        for i in range(len(route) - 1):
            from_node = route[i]
            to_node = route[i + 1]
            total_distance += distances[from_node][to_node]

        ranges.append(total_distance)

    return ranges

def main():
    file_path = "10 2.txt"
    n, k, distances = read_data(file_path)

    start_time = time()
    solution = min_max_vrp(n, k, distances)
    end_time = time()

    if solution:
        print(f"Solution found in {end_time - start_time:.2f} seconds")
        print(format_solution(solution))
        ranges = calculate_vehicle_ranges(solution, distances)
        max_range = max(ranges)
        print(f"Max range: {max_range}")
        print(f"Vehicle ranges: {ranges}")
    else:
        print("No solution found")



if __name__ == "__main__":
    main()