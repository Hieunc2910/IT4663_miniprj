# PYTHON
import sys
import heapq
from time import time


def read_data():
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
        self.depth = 0

    def __lt__(self, other):
        if abs(self.lower_bound - other.lower_bound) < 1e-6:
            return self.depth > other.depth
        return self.lower_bound < other.lower_bound

    def clone(self):
        new_state = State(self.n, self.k)
        new_state.routes = [route[:] for route in self.routes]
        new_state.distances = self.distances.copy()
        new_state.unvisited = self.unvisited.copy()
        new_state.lower_bound = self.lower_bound
        new_state.depth = self.depth + 1
        return new_state


def preprocess_distances(distances, n):
    min_outgoing = [float('inf')] * (n + 1)
    for i in range(n + 1):
        for j in range(n + 1):
            if i != j:
                min_outgoing[i] = min(min_outgoing[i], distances[i][j])
    return min_outgoing


def greedy_solution(n, k, distances):
    state = State(n, k)
    for i in range(k):
        state.routes[i].append(0)

    nodes = sorted(range(1, n + 1), key=lambda x: distances[0][x])

    for node in nodes:
        vehicle_id = min(range(k), key=lambda v: state.distances[v])
        last_node = state.routes[vehicle_id][-1]
        state.routes[vehicle_id].append(node)
        state.distances[vehicle_id] += distances[last_node][node]

    return state, max(state.distances)


def compute_lower_bound(state, distances, min_outgoing):
    current_max = max(state.distances) if state.distances else 0

    if not state.unvisited:
        return current_max

    min_insertion_cost = float('inf')
    for node in state.unvisited:
        best_insertion = float('inf')
        for vehicle_id in range(state.k):
            last_node = state.routes[vehicle_id][-1]
            insertion_cost = distances[last_node][node]
            best_insertion = min(best_insertion, insertion_cost)
        min_insertion_cost = min(min_insertion_cost, best_insertion)

    mst_cost = 0
    for node in state.unvisited:
        mst_cost += min_outgoing[node]

    remaining_nodes = len(state.unvisited)
    min_vehicles_needed = remaining_nodes // state.k + (1 if remaining_nodes % state.k else 0)

    return max(current_max,
               current_max + min_insertion_cost,
               min(state.distances) + mst_cost / state.k)


def min_max_vrp(n, k, distances):
    start_time = time()
    timeout = 25

    min_outgoing = preprocess_distances(distances, n)
    initial_solution, best_max_distance = greedy_solution(n, k, distances)
    best_solution = initial_solution

    root = State(n, k)
    for i in range(k):
        root.routes[i].append(0)
    root.lower_bound = compute_lower_bound(root, distances, min_outgoing)

    queue = [root]
    nodes_explored = 0
    best_states = {}

    while queue and nodes_explored < 1000000 and time() - start_time < timeout:
        nodes_explored += 1
        current = heapq.heappop(queue)

        unvisited_key = frozenset(current.unvisited)
        if unvisited_key in best_states:
            max_dist = max(current.distances)
            if max_dist >= best_states[unvisited_key]:
                continue
            best_states[unvisited_key] = max_dist
        else:
            best_states[unvisited_key] = max(current.distances)

        if not current.unvisited:

            max_distance = 0
            max_distance = max(current.distances)

            if max_distance < best_max_distance:
                best_max_distance = max_distance
                best_solution = current
            continue

        if current.lower_bound >= best_max_distance:
            continue

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

        insertion_costs.sort()
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

    return best_solution


def format_solution(solution):
    output = [str(solution.k)]
    for route in solution.routes:
        if route[-1] == 0:
            route = route[:-1]
        output.append(str(len(route)))
        output.append(' '.join(map(str, route)))
    return '\n'.join(output)


def main():
    n, k, distances = read_data()
    solution = min_max_vrp(n, k, distances)
    if solution:
        print(format_solution(solution))


if __name__ == "__main__":
    main()

