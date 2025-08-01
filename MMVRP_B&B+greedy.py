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
        self.n, self.k = n, k
        self.routes = [[] for _ in range(k)]
        self.distances = [0] * k
        self.unvisited = set(range(1, n + 1))
        self.lower_bound = 0
        self.depth = 0

    def __lt__(self, other):
        return (self.lower_bound, -self.depth) < (other.lower_bound, -other.depth)

    def clone(self):
        new_state = State(self.n, self.k)
        new_state.routes = [route[:] for route in self.routes]
        new_state.distances = self.distances.copy()
        new_state.unvisited = self.unvisited.copy()
        new_state.lower_bound = self.lower_bound
        new_state.depth = self.depth + 1
        return new_state


def preprocess_distances(distances, n):
    min_outgoing = [min(distances[i][j] for j in range(n + 1) if i != j) for i in range(n + 1)]
    min_incoming = [min(distances[i][j] for i in range(n + 1) if i != j) for j in range(n + 1)]
    return min_outgoing, min_incoming


def greedy_solve(n, k, distances):
    """Min-max greedy algorithm"""
    state = State(n, k)
    for i in range(k):
        state.routes[i].append(0)

    while state.unvisited:
        best_cost, best_node, best_vehicle = float('inf'), None, None

        for node in state.unvisited:
            for vehicle_id in range(k):
                last_node = state.routes[vehicle_id][-1]
                new_distance = state.distances[vehicle_id] + distances[last_node][node]
                max_distance = max(max(d for i, d in enumerate(state.distances) if i != vehicle_id), new_distance)

                if max_distance < best_cost:
                    best_cost, best_node, best_vehicle = max_distance, node, vehicle_id

        if best_node and best_vehicle is not None:
            last_node = state.routes[best_vehicle][-1]
            state.routes[best_vehicle].append(best_node)
            state.distances[best_vehicle] += distances[last_node][best_node]
            state.unvisited.remove(best_node)

    return state, max(state.distances)


def compute_lower_bound(state, distances, min_outgoing, min_incoming):
    """Enhanced lower bound with multiple strategies"""
    if not state.unvisited:
        return max(state.distances) if state.distances else 0

    current_max = max(state.distances) if state.distances else 0
    bounds = [current_max]

    # Minimum insertion cost bound
    min_insertion = min(
        min(distances[state.routes[v][-1]][node] for v in range(state.k))
        for node in state.unvisited
    )
    bounds.append(current_max + min_insertion)

    # MST-based bound
    mst_cost = sum(min_outgoing[node] for node in state.unvisited)
    bounds.append(min(state.distances) + mst_cost / state.k)

    # Depot-based bound
    for vehicle_id in range(state.k):
        last_node = state.routes[vehicle_id][-1]
        min_to_unvisited = min(distances[last_node][node] for node in state.unvisited)
        bounds.append(state.distances[vehicle_id] + min_to_unvisited)

    # Load balancing bound
    total_remaining = sum(min_incoming[node] for node in state.unvisited)
    bounds.append(min(state.distances) + total_remaining / state.k)

    return max(bounds)


def min_max_vrp(n, k, distances):
    start_time = time()
    timeout = 300

    min_outgoing, min_incoming = preprocess_distances(distances, n)
    initial_solution, best_max_distance = greedy_solve(n, k, distances)
    best_solution = initial_solution

    root = State(n, k)
    for i in range(k):
        root.routes[i].append(0)
    root.lower_bound = compute_lower_bound(root, distances, min_outgoing, min_incoming)

    queue, nodes_explored, best_states = [root], 0, {}

    while queue and nodes_explored < 1000000 and time() - start_time < timeout:
        nodes_explored += 1
        current = heapq.heappop(queue)

        # State pruning
        state_key = (frozenset(current.unvisited), tuple(sorted(current.distances)))
        if state_key in best_states and max(current.distances) >= best_states[state_key]:
            continue
        best_states[state_key] = max(current.distances)

        if not current.unvisited:
            max_distance = max(current.distances)
            if max_distance < best_max_distance:
                best_max_distance, best_solution = max_distance, current
            continue

        if current.lower_bound >= best_max_distance:
            continue

        # Smart branching
        remaining = len(current.unvisited)
        max_nodes_branch = 2 if remaining > 10 else 3 if remaining > 5 else remaining
        max_vehicles_per_node = 1 if remaining > 10 else 2 if remaining > 5 else k

        # Find best insertions for each node
        node_costs = []
        for node in current.unvisited:
            vehicle_costs = []
            for vehicle_id in range(k):
                last_node = current.routes[vehicle_id][-1]
                cost = distances[last_node][node]
                new_total = current.distances[vehicle_id] + cost
                max_after = max(max(d for i, d in enumerate(current.distances) if i != vehicle_id), new_total)
                vehicle_costs.append((max_after, vehicle_id, new_total))

            vehicle_costs.sort()
            min_cost = vehicle_costs[0][0]
            node_costs.append((min_cost, node, vehicle_costs[:max_vehicles_per_node]))

        node_costs.sort()

        for _, node_id, vehicle_options in node_costs[:max_nodes_branch]:
            for max_after, vehicle_id, new_total in vehicle_options:
                if new_total >= best_max_distance:
                    continue

                new_state = current.clone()
                new_state.unvisited.remove(node_id)
                new_state.routes[vehicle_id].append(node_id)
                new_state.distances[vehicle_id] = new_total

                if max(new_state.distances) >= best_max_distance:
                    continue

                new_state.lower_bound = compute_lower_bound(new_state, distances, min_outgoing, min_incoming)
                if new_state.lower_bound < best_max_distance:
                    heapq.heappush(queue, new_state)

    return best_solution


def format_solution(solution):
    output = [str(solution.k)]
    for route in solution.routes:
        if route and route[-1] == 0:
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